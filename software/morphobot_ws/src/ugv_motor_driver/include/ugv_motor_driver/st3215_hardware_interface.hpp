#ifndef HYBRID_ROBOT_HARDWARE__ST3215_HARDWARE_INTERFACE_HPP_
#define HYBRID_ROBOT_HARDWARE__ST3215_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ugv_motor_driver/srv/read_positions.hpp"
#include "ugv_motor_driver/srv/write_positions.hpp"
#include "ugv_motor_driver/srv/write_velocities.hpp"
#include "ugv_motor_driver/srv/read_velocities.hpp"

namespace ugv_motor_driver
{

class ST3215HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ST3215HardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ROS 2 node for service clients
  rclcpp::Node::SharedPtr node_;
  
  // Service clients
  rclcpp::Client<srv::ReadPositions>::SharedPtr read_client_;
  rclcpp::Client<srv::WritePositions>::SharedPtr write_client_;
  rclcpp::Client<srv::WriteVelocities>::SharedPtr write_velocities_client_;
  rclcpp::Client<srv::ReadVelocities>::SharedPtr read_velocities_client_;
  
  // Parameters
  std::string serial_port_;
  std::vector<int> servo_ids_;
  std::map<int, size_t> servo_to_joint_map_;  // Maps servo ID to joint index
  
  // State storage
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  
  // Command storage
  std::vector<double> hw_position_commands_;
  std::vector<double> hw_velocity_commands_;
  
  // Helper functions
  double ticks_to_radians(int ticks);
  int radians_to_ticks(double radians);
};

}  // namespace ugv_motor_driver

#endif  // HYBRID_ROBOT_HARDWARE__ST3215_HARDWARE_INTERFACE_HPP_
