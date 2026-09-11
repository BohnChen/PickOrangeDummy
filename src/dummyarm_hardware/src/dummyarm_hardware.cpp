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
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
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
  char buf[256];
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(fd_, &fds);
  struct timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
  int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
  if (ret <= 0) { return false; }
  int n = (int)::read(fd_, buf, sizeof(buf) - 1);
  if (n <= 0) { return false; }
  buf[n] = 0;
  std::string chunk(buf);
  auto pos = chunk.find_first_of("\r\n");
  if (pos != std::string::npos) {
    line = chunk.substr(0, pos);
  } else {
    line = chunk;
  }
  return !line.empty();
}

// ------------------------------------------------------------------ Hardware
DummyArmHardware::DummyArmHardware()
: joints_({
    Joint("Joint1"), Joint("Joint2"), Joint("Joint3"),
    Joint("Joint4"), Joint("Joint5"), Joint("Joint6")}) {}

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

  // 创建使能与失能管理服务节点
  service_node_ = std::make_shared<rclcpp::Node>("dummyarm_hardware_service_helper");
  enable_srv_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "/dummyarm/enable",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      if (enable_hardware()) {
        response->success = true;
        response->message = "机械臂已成功使能 (!START)! 电机闭环锁轴就绪。";
        RCLCPP_INFO(logger_, "【硬件服务】收到使能请求: 机械臂已成功使能 (!START)");
      } else {
        response->success = false;
        response->message = "使能失败，串口未响应或主控拒绝。";
        RCLCPP_ERROR(logger_, "【硬件服务】使能失败: 串口通信异常");
      }
    });

  disable_srv_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "/dummyarm/disable",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      disable_hardware();
      response->success = true;
      response->message = "机械臂已安全失能 (!STOP)! 电机力矩已断开。";
      RCLCPP_INFO(logger_, "【硬件服务】收到失能请求: 机械臂已失能 (!STOP)");
    });

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(service_node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  RCLCPP_INFO(logger_, "【硬件管理】使能服务已就绪: /dummyarm/enable, /dummyarm/disable (当前处于安全失能状态)");
  return CallbackReturn::SUCCESS;
}

bool DummyArmHardware::enable_hardware()
{
  if (!serial_.write("!START\n")) {
    return false;
  }
  is_enabled_ = true;
  return true;
}

bool DummyArmHardware::disable_hardware()
{
  serial_.write("!STOP\n");
  is_enabled_ = false;
  return true;
}

CallbackReturn DummyArmHardware::on_activate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn DummyArmHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  disable_hardware();
  return CallbackReturn::SUCCESS;
}

CallbackReturn DummyArmHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  disable_hardware();
  if (executor_) {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    executor_.reset();
  }
  service_node_.reset();
  serial_.close();
  return CallbackReturn::SUCCESS;
}

CallbackReturn DummyArmHardware::on_shutdown(const rclcpp_lifecycle::State &)
{
  disable_hardware();
  if (executor_) {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    executor_.reset();
  }
  service_node_.reset();
  serial_.close();
  return CallbackReturn::SUCCESS;
}

bool DummyArmHardware::query_positions(double period_s)
{
  std::string line;
  if (!serial_.write("#GETJPOS\n")) {
    return false;
  }
  if (!serial_.readLine(line, 300)) {
    return false;
  }
  // 期望: "ok j1 j2 j3 j4 j5 j6" (度)
  if (line.rfind("ok ", 0) != 0) {
    return false;
  }
  float v[6];
  if (std::sscanf(line.c_str(), "ok %f %f %f %f %f %f",
                  &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
    return false;
  }
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

bool DummyArmHardware::send_commands()
{
  if (!is_enabled_) {
    // 未通过 /dummyarm/enable 手动使能前，绝不下发任何运动命令，确保物理绝对安全
    return true;
  }

  // 仅当有命令下发时按 '>' 格式发出 (INT 模式, 速度=命令速度参数化)
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
  return serial_.write(std::string(buf, n));
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
