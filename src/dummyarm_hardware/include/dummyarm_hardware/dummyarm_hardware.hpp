#ifndef DUMMYARM_HARDWARE_DUMMYARM_HARDWARE_HPP_
#define DUMMYARM_HARDWARE_DUMMYARM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace dummyarm_hardware
{

// FW ↔ CAD 模型角度映射:
//   固件 REST: [0, -73, 180, 0, 0, 0] (度) -> CAD REST: [0, 0, 0, 0, 0, 0] (度)
//   固件 HOME: [0,   0,  90, 0, 0, 0] (度) -> CAD HOME: [0, 73, -90, 0, 0, 0] (度)
//
//   q_cad[1] = q_fw[1] + 73.0
//   q_cad[2] = q_fw[2] - 180.0
//   其余关节同向等值 (1:1)

inline double map_fw_to_model(double q_fw_deg, size_t i)
{
  switch (i) {
    case 0: return q_fw_deg;
    case 1: return q_fw_deg + 73.0;
    case 2: return q_fw_deg - 180.0;
    case 3: return q_fw_deg;
    case 4: return q_fw_deg;
    case 5: return q_fw_deg;
    default: return q_fw_deg;
  }
}

inline double map_model_to_fw(double q_model_deg, size_t i)
{
  switch (i) {
    case 0: return q_model_deg;
    case 1: return q_model_deg - 73.0;
    case 2: return q_model_deg + 180.0;
    case 3: return q_model_deg;
    case 4: return q_model_deg;
    case 5: return q_model_deg;
    default: return q_model_deg;
  }
}

// 物理固件层级严格硬限位 (度) - 超出任何一点立即在驱动级拒绝发送
struct JointHardLimit {
  double min_deg;
  double max_deg;
};

static const JointHardLimit FW_HARD_LIMITS[6] = {
  {-170.0, 170.0}, // J1
  {-74.0,   92.0}, // J2: 物理 -73° (REST) 到 +90°
  { 35.0,  182.0}, // J3: 严禁低于 35°！(35° 为机构防折断硬红线, 180° 为 REST)
  {-180.0, 180.0}, // J4
  {-95.0,   95.0}, // J5: 碰撞极限 ±95° (超过105°与J4壳体物理干涉碰撞)
  {-720.0, 720.0}  // J6: 旋切轴
};

inline double deg2rad(double d) { return d * M_PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / M_PI; }

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
  mutable std::string rx_buf_;
};

class DummyArmHardware : public hardware_interface::SystemInterface
{
public:
  DummyArmHardware();
  virtual ~DummyArmHardware();

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
  float joint_speed_ = 15.0f;  // '>' 命令速度 (平稳安全的 15°/s 速度)
  bool prev_set_ = false;
  std::vector<double> last_vel_;
  std::vector<double> last_sent_cmd_;
  rclcpp::Logger logger_ = rclcpp::get_logger("dummyarm_hardware");
  rclcpp::Clock clock_{RCL_SYSTEM_TIME};
  bool is_enabled_{false};
};

}  // namespace dummyarm_hardware

#endif  // DUMMYARM_HARDWARE_DUMMYARM_HARDWARE_HPP_
