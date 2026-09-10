#ifndef DUMMYARM_HARDWARE_DUMMYARM_HARDWARE_HPP_
#define DUMMYARM_HARDWARE_DUMMYARM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dummyarm_hardware
{

// FW→模型 角度映射 (与 bridge 一致):
//   q_model = SIGNS[i]·(q_fw + TH_HOME[i])   [度]
// 2026-09-10: 真值链几何按 DH 修正后 J3 零位平移 90°, TH_HOME[2]: 90 -> 0
static const double SIGNS[6] = {1, -1, -1, 1, 1, 1};
static const double TH_HOME[6] = {0, -90, 0, 0, 0, 0};

inline double map_fw_to_model(double q_fw_deg, size_t i)
{
  return SIGNS[i] * (q_fw_deg + TH_HOME[i]);
}
inline double map_model_to_fw(double q_model, size_t i)
{
  return q_model / SIGNS[i] - TH_HOME[i];
}
inline double deg2rad(double d) { return d * M_PI / 180.0; }

class SerialPort
{
public:
  SerialPort();
  bool open(const std::string & device, int baud);
  void close();
  bool write(const std::string & s) const;
  bool readLine(std::string & line, int timeout_ms) const;

private:
  int fd_;
};

class DummyArmHardware : public hardware_interface::SystemInterface
{
public:
  DummyArmHardware();

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type
  read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type
  write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Joint
  {
    explicit Joint(const std::string & name) : name_(name) {}
    std::string name_;
    double measured_position_ = 0.0;
    double measured_velocity_ = 0.0;
    double command_position_ = 0.0;
  };

  bool query_positions(double period_s);
  bool send_commands();

public:
  SerialPort serial_;
  std::vector<Joint> joints_;
  std::string port_;
  double rate_ = 50.0;
  float joint_speed_ = 10.0f;   // '>' 命令速度 (0~100, 低速安全默认)
  bool prev_set_ = false;
  std::vector<double> last_vel_;
  rclcpp::Logger logger_ = rclcpp::get_logger("dummyarm_hardware");
};

}  // namespace dummyarm_hardware

#endif  // DUMMYARM_HARDWARE_DUMMYARM_HARDWARE_HPP_
