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
  if (is_enabled_) {
    serial_.write("!STOP\n");
    serial_.write("!DISABLE\n");
    is_enabled_ = false;
    RCLCPP_INFO(logger_, "【硬件驱动】机械臂电机已失能 (!STOP / !DISABLE)，力矩已释放");
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DummyArmHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (is_enabled_) {
    serial_.write("!STOP\n");
    serial_.write("!DISABLE\n");
    is_enabled_ = false;
  }
  serial_.close();
  return CallbackReturn::SUCCESS;
}

CallbackReturn DummyArmHardware::on_shutdown(const rclcpp_lifecycle::State &)
{
  if (is_enabled_) {
    serial_.write("!STOP\n");
    serial_.write("!DISABLE\n");
    is_enabled_ = false;
  }
  serial_.close();
  return CallbackReturn::SUCCESS;
}

CallbackReturn DummyArmHardware::on_error(const rclcpp_lifecycle::State &)
{
  if (is_enabled_) {
    serial_.write("!STOP\n");
    serial_.write("!DISABLE\n");
    is_enabled_ = false;
  }
  serial_.close();
  return CallbackReturn::SUCCESS;
}

bool DummyArmHardware::query_positions(double period_s)
{
  if (!serial_.write("#GETJPOS\n")) {
    return true; // 偶发写入失败平滑容错
  }

  std::string line;
  // 2026-11-01 修正：原超时 2ms 太短。
  //   115200 下一条 "ok 0.00 -73.00 180.00 -0.00 0.00 0.00\n" 约 38 字符 ≈ 3.3ms，
  //   2ms 必然读不全；读不全 → 回包堆积在固件 CDC 发送缓冲 → 固件阻塞写 → 串口彻底哑死
  //   （表现为 /joint_states 永远是 0，且必须断电重启才能恢复）。
  // 改成: 每次 12ms、最多 3 次尝试（正常一次就返回），失败不再 break 而是 continue。
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (!serial_.readLine(line, 12)) {
      continue;
    }
    // 过滤可能漏掉的孤立 "ok" 或空行
    if (line == "ok" || line.empty()) {
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
            double delta = q_model - prev;
            // 传感器微步底噪滤波：变化小于 0.03° (~0.0005 rad) 认定为真正静止，速度干净归零
            if (std::abs(delta) < 0.0005) {
              joints_[i].measured_velocity_ = 0.0;
            } else {
              joints_[i].measured_velocity_ = delta / period_s;
            }
          }
          joints_[i].measured_position_ = q_model;
        }
        prev_set_ = true;
        return true;
      }
    }
  }

  // 三次都没拿到完整行：清掉残留的半截数据，避免下次把旧字节拼进新回包
  serial_.clearRx();
  return true;
}

bool DummyArmHardware::send_commands()
{
  if (!is_enabled_) {
    return true;
  }

  // 静止检测：如果所有关节指令与上次发送完全一致（已停止），静默不发
  if (last_sent_cmd_.size() == joints_.size()) {
    bool same = true;
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (std::abs(joints_[i].command_position_ - last_sent_cmd_[i]) > 1e-5) {
        same = false;
        break;
      }
    }
    if (same) {
      return true;
    }
  }

  // 准备下发指令 (平滑、连续、无延迟透传)
  float j[6];
  for (size_t i = 0; i < joints_.size(); ++i) {
    double target_pos = joints_[i].command_position_;

    // 核心架构改进：若开启了轨迹阶段 J6 锁定，J6 物理指令完全隔离为专用的切削位置！
    if (i == 5 && lock_j6_during_trajectory_) {
      target_pos = cutter_target_pos_;
    }

    double cmd_deg = target_pos * 180.0 / M_PI;
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
    // 纯异步写：字符直接入内核缓冲区后立即返回，绝不阻塞控制器线程
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
