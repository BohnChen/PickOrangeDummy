#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cstdio>
#include <string>
#include <cmath>
#include <array>
#include <optional>
#include <sys/select.h>
#include <sys/ioctl.h>

#include "dummyarm_hardware/dummyarm_hardware.hpp"

namespace dummyarm_hardware
{

using hardware_interface::return_type;
using hardware_interface::CallbackReturn;

static void set_dtr_low(int fd)
{
  int flags = TIOCM_DTR;
  ioctl(fd, TIOCMBIC, &flags);
}

// ------------------------------------------------------------------ Serial
SerialPort::SerialPort() : fd_(-1) {}

bool SerialPort::open(const std::string & device, int baud)
{
  (void)baud;
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }
  struct termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_); fd_ = -1; return false;
  }
  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);
  tty.c_cflag = CS8 | CLOCAL | CREAD;
  tty.c_cflag &= ~HUPCL;  // 绝不在关闭串口时释放/挂起 DTR，避免触发 STM32 复位！
  tty.c_iflag = IGNPAR | IGNBRK;
  tty.c_oflag = 0;
  tty.c_lflag = 0;
  tty.c_cc[VTIME] = 1;    // 0.1s timeout
  tty.c_cc[VMIN] = 0;
  tcsetattr(fd_, TCSANOW, &tty);
  // 拉低 DTR, 防止复位 REF 主控(重要!)
  set_dtr_low(fd_);
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    tcdrain(fd_); // 等待所有指令字符发送完毕
    set_dtr_low(fd_); // 维持 DTR 为低
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::write(const std::string & s) const
{
  if (fd_ < 0) { return false; }
  size_t n = ::write(fd_, s.data(), s.size());
  return n == s.size();
}

bool SerialPort::readLine(std::string & line, int timeout_ms) const
{
  line.clear();
  if (fd_ < 0) { return false; }

  // 1. 如果缓冲区里已经有完整的一行，直接弹出返回
  auto pos = rx_buf_.find_first_of("\r\n");
  if (pos != std::string::npos) {
    line = rx_buf_.substr(0, pos);
    // 丢弃连续的回车换行符
    while (pos < rx_buf_.size() && (rx_buf_[pos] == '\r' || rx_buf_[pos] == '\n')) {
      pos++;
    }
    rx_buf_.erase(0, pos);
    if (!line.empty()) { return true; }
  }

  // 2. 否则通过 select 读取新数据
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(fd_, &fds);
  struct timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
  int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
  if (ret <= 0) { return false; }

  char buf[256];
  int n = (int)::read(fd_, buf, sizeof(buf) - 1);
  if (n <= 0) { return false; }
  buf[n] = 0;
  rx_buf_.append(buf, n);

  // 再次查找换行
  pos = rx_buf_.find_first_of("\r\n");
  if (pos != std::string::npos) {
    line = rx_buf_.substr(0, pos);
    while (pos < rx_buf_.size() && (rx_buf_[pos] == '\r' || rx_buf_[pos] == '\n')) {
      pos++;
    }
    rx_buf_.erase(0, pos);
    return !line.empty();
  }
  return false;
}

// ------------------------------------------------------------------ Hardware
DummyArmHardware::DummyArmHardware()
: joints_({
    Joint("Joint1"), Joint("Joint2"), Joint("Joint3"),
    Joint("Joint4"), Joint("Joint5"), Joint("Joint6")}) {}

DummyArmHardware::~DummyArmHardware()
{
  if (is_enabled_) {
    serial_.write("!STOP\n");
    usleep(50000);
    serial_.write("!DISABLE\n");
    usleep(50000);
    is_enabled_ = false;
  }
  serial_.close();
}

CallbackReturn DummyArmHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  port_ = info_.hardware_parameters.count("port") ? info_.hardware_parameters.at("port")
                                                  : "/dev/ttyACM1";
  rate_ = info_.hardware_parameters.count("rate") ? std::stod(info_.hardware_parameters.at("rate"))
                                                  : 50.0;
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DummyArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (auto & j : joints_) {
    si.emplace_back(j.name_, "position", &j.measured_position_);
    si.emplace_back(j.name_, "velocity", &j.measured_velocity_);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface> DummyArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (auto & j : joints_) {
    ci.emplace_back(j.name_, "position", &j.command_position_);
  }
  return ci;
}

CallbackReturn DummyArmHardware::on_configure(const rclcpp_lifecycle::State &)
{
  if (!serial_.open(port_, 115200)) {
    RCLCPP_ERROR(logger_, "无法打开串口 %s", port_.c_str());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "【硬件驱动】串口 %s 打开成功 (115200 8N1)", port_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn DummyArmHardware::on_activate(const rclcpp_lifecycle::State &)
{
  // 核心安全生命周期：控制器激活时安全使能机械臂电机，建立保持力矩
  if (serial_.write("!START\n")) {
    is_enabled_ = true;
    RCLCPP_INFO(logger_, "【硬件驱动】机械臂电机已使能 (!START)，闭环锁轴就绪");
  } else {
    RCLCPP_WARN(logger_, "【硬件驱动】未能发送 !START 使能命令");
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DummyArmHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  // 控制器失活时释放力矩，双保险失能
  serial_.write("!STOP\n");
  usleep(50000); // 50ms
  serial_.write("!DISABLE\n");
  usleep(50000);
  is_enabled_ = false;
  RCLCPP_INFO(logger_, "【硬件驱动】机械臂电机已失能 (!STOP / !DISABLE)，力矩已释放");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DummyArmHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  serial_.write("!STOP\n");
  usleep(50000);
  serial_.write("!DISABLE\n");
  usleep(50000);
  is_enabled_ = false;
  serial_.close();
  return CallbackReturn::SUCCESS;
}

CallbackReturn DummyArmHardware::on_shutdown(const rclcpp_lifecycle::State &)
{
  serial_.write("!STOP\n");
  usleep(50000);
  serial_.write("!DISABLE\n");
  usleep(50000);
  is_enabled_ = false;
  serial_.close();
  return CallbackReturn::SUCCESS;
}

bool DummyArmHardware::query_positions(double period_s)
{
  if (!serial_.write("#GETJPOS\n")) {
    return true; // 即使偶发写入失败，也不中断周期
  }

  std::string line;
  // 最多尝试 2 次（过滤前一次命令残留的纯 "ok"）
  for (int attempt = 0; attempt < 2; ++attempt) {
    if (!serial_.readLine(line, 10)) { // 严格限制在 10ms 以内，绝不阻塞 50ms 周期！
      break;
    }
    // 如果是纯 "ok" (命令确认)，继续读下一行才是 #GETJPOS 回复
    if (line == "ok") {
      continue;
    }
    if (line.rfind("ok ", 0) == 0) {
      float v[6];
      if (std::sscanf(line.c_str(), "ok %f %f %f %f %f %f",
                      &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) == 6) {
        last_vel_.resize(joints_.size());
        for (size_t i = 0; i < joints_.size(); ++i) {
          double q_model_deg = map_fw_to_model(v[i], i);
          double q_model = deg2rad(q_model_deg);
          double prev = joints_[i].measured_position_;
          if (period_s > 1e-9 && prev_set_) {
            joints_[i].measured_velocity_ = (q_model - prev) / period_s;
          }
          joints_[i].measured_position_ = q_model;
        }
        prev_set_ = true;
        return true;
      }
    }
  }

  // 若本次偶尔未收到完整行，平滑沿用上一帧数据，保证 controller_manager 不会发生 Overrun
  return true;
}

bool DummyArmHardware::send_commands()
{
  if (!is_enabled_) {
    return true;
  }

  // 防抖优化：如果机械臂静止处于目标位，不向下位机高频刷发相同的重复指令
  if (last_sent_cmd_.size() == joints_.size()) {
    bool changed = false;
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (std::abs(joints_[i].command_position_ - last_sent_cmd_[i]) > 0.0005) { // ~0.03°
        changed = true;
        break;
      }
    }
    if (!changed) {
      return true; // 位置未发生变化，静默保持，释放串口总线带宽
    }
  }

  // 仅当有命令下发且发生变化时按 '>' 格式发出 (INT 模式, 速度=命令速度参数化)
  float j[6];
  for (size_t i = 0; i < joints_.size(); ++i) {
    double cmd_deg = joints_[i].command_position_ * 180.0 / M_PI;
    double fw_deg = map_model_to_fw(cmd_deg, i);

    // 核心安全硬拦截：超出物理固件硬限位立即拦截并拒绝下发！
    if (fw_deg < FW_HARD_LIMITS[i].min_deg || fw_deg > FW_HARD_LIMITS[i].max_deg) {
      RCLCPP_ERROR_THROTTLE(
        logger_, clock_, 1000,
        "【安全硬拦截】关节 %zu 指令越界被丢弃! 目标固件角度: %.2f°, 允许物理区间: [%.2f°, %.2f°]",
        i + 1, fw_deg, FW_HARD_LIMITS[i].min_deg, FW_HARD_LIMITS[i].max_deg);
      return false; // 拒绝下发非法指令，保护机械臂
    }
    j[i] = (float)fw_deg;
  }
  char buf[160];
  int n = std::snprintf(buf, sizeof(buf), ">%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                        j[0], j[1], j[2], j[3], j[4], j[5], joint_speed_);
  if (n <= 0) { return false; }
  
  if (serial_.write(std::string(buf, n))) {
    last_sent_cmd_.resize(joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i) {
      last_sent_cmd_[i] = joints_[i].command_position_;
    }
    return true;
  }
  return false;
}

return_type DummyArmHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  double period_s = period.seconds();
  query_positions(period_s);
  return return_type::OK;
}

return_type DummyArmHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  send_commands();
  return return_type::OK;
}

}  // namespace dummyarm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dummyarm_hardware::DummyArmHardware,
                       hardware_interface::SystemInterface)
