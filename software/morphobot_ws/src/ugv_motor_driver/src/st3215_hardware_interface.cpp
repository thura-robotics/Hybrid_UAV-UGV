#include "ugv_motor_driver/st3215_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace ugv_motor_driver
{

hardware_interface::CallbackReturn ST3215HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  //Call parent class initialization
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  serial_port_ = info_.hardware_parameters["serial_port"];
  
  // Parse servo IDs
  std::string servo_ids_str = info_.hardware_parameters["servo_ids"];
  std::stringstream ss(servo_ids_str);
  std::string item;
  while (std::getline(ss, item, ',')) {
    servo_ids_.push_back(std::stoi(item));
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ST3215HardwareInterface"),
    "Initializing hardware interface with serial port: %s", serial_port_.c_str());
  
  RCLCPP_INFO(
    rclcpp::get_logger("ST3215HardwareInterface"),
    "Number of servos: %zu", servo_ids_.size());

  // Build mapping from servo ID to joint index
  // This allows non-sequential servo IDs (e.g., 1,2,3,9,10)
  for (size_t i = 0; i < servo_ids_.size(); i++) {
    servo_to_joint_map_[servo_ids_[i]] = i;
    RCLCPP_INFO(
      rclcpp::get_logger("ST3215HardwareInterface"),
      "Mapped servo ID %d to joint index %zu", servo_ids_[i], i);
  }

  // Resize state and command vectors
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocity_commands_.resize(info_.joints.size(), 0.0);

  // Verify joint configuration
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Check that joint has at least one command interface (position or velocity)
    if (joint.command_interfaces.empty())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ST3215HardwareInterface"),
        "Joint '%s' has no command interfaces.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Verify command interfaces are either position or velocity
    for (const auto & cmd_interface : joint.command_interfaces)
    {
      if (cmd_interface.name != hardware_interface::HW_IF_POSITION &&
          cmd_interface.name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("ST3215HardwareInterface"),
          "Joint '%s' has unsupported command interface '%s'.",
          joint.name.c_str(), cmd_interface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ST3215HardwareInterface"),
        "Joint '%s' has %zu state interfaces. 3 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ST3215HardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ST3215HardwareInterface"), "Configuring hardware interface...");

  // Create ROS 2 node for service clients
  node_ = rclcpp::Node::make_shared("st3215_hardware_interface_client");
  
  // Create service clients
  read_client_ = node_->create_client<srv::ReadPositions>("st3215/read_positions");
  write_client_ = node_->create_client<srv::WritePositions>("st3215/write_positions");
  write_velocities_client_ = node_->create_client<srv::WriteVelocities>("st3215/write_velocities");
  read_velocities_client_ = node_->create_client<srv::ReadVelocities>("st3215/read_velocities");
  
  // Wait for services to be available
  RCLCPP_INFO(rclcpp::get_logger("ST3215HardwareInterface"), "Waiting for ST3215 services...");
  
  if (!read_client_->wait_for_service(10s)) {
    RCLCPP_ERROR(rclcpp::get_logger("ST3215HardwareInterface"), 
                 "Service /st3215/read_positions not available!");
    RCLCPP_ERROR(rclcpp::get_logger("ST3215HardwareInterface"),
                 "Make sure st3215_service_node is running!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  if (!write_client_->wait_for_service(10s)) {
    RCLCPP_ERROR(rclcpp::get_logger("ST3215HardwareInterface"),
                 "Service /st3215/write_positions not available!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  if (!write_velocities_client_->wait_for_service(10s)) {
    RCLCPP_ERROR(rclcpp::get_logger("ST3215HardwareInterface"),
                 "Service /st3215/write_velocities not available!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  if (!read_velocities_client_->wait_for_service(10s)) {
    RCLCPP_ERROR(rclcpp::get_logger("ST3215HardwareInterface"),
                 "Service /st3215/read_velocities not available!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("ST3215HardwareInterface"), "Connected to ST3215 services!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ST3215HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ST3215HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // Export command interfaces based on what's defined in URDF
    for (const auto & cmd_interface : info_.joints[i].command_interfaces)
    {
      if (cmd_interface.name == hardware_interface::HW_IF_POSITION)
      {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]));
      }
      else if (cmd_interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]));
      }
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ST3215HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ST3215HardwareInterface"), "Activating hardware interface...");

  // Read initial positions (note: velocity-mode servos may return 0)
  auto request = std::make_shared<srv::ReadPositions::Request>();
  request->servo_ids = servo_ids_;
  
  auto result = read_client_->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(node_, result, 5s) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result.get();
    if (response->positions.size() == servo_ids_.size())
    {
      for (size_t i = 0; i < servo_ids_.size(); i++)
      {
        int servo_id = servo_ids_[i];
        size_t joint_idx = servo_to_joint_map_[servo_id];
        
        hw_positions_[joint_idx] = ticks_to_radians(response->positions[i]);
        hw_position_commands_[joint_idx] = hw_positions_[joint_idx];
        hw_velocities_[joint_idx] = 0.0;
        hw_velocity_commands_[joint_idx] = 0.0;
        hw_efforts_[joint_idx] = 0.0;
        
        RCLCPP_INFO(
          rclcpp::get_logger("ST3215HardwareInterface"),
          "Servo %d (joint index %zu) initial position: %.3f rad", 
          servo_id, joint_idx, hw_positions_[joint_idx]);
      }
      
      if (!response->success)
      {
        RCLCPP_WARN(rclcpp::get_logger("ST3215HardwareInterface"),
                    "Position read reported issues: %s (continuing anyway)", response->message.c_str());
      }
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("ST3215HardwareInterface"),
                   "Position array size mismatch: expected %zu, got %zu", 
                   servo_ids_.size(), response->positions.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("ST3215HardwareInterface"),
                 "Timeout reading initial positions");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("ST3215HardwareInterface"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ST3215HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ST3215HardwareInterface"), "Deactivating hardware interface...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ST3215HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read positions
  auto pos_request = std::make_shared<srv::ReadPositions::Request>();
  pos_request->servo_ids = servo_ids_;
  
  auto pos_future = read_client_->async_send_request(pos_request);
  
  if (rclcpp::spin_until_future_complete(node_, pos_future, 100ms) == 
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = pos_future.get();
    if (response->success && response->positions.size() == servo_ids_.size())
    {
      for (size_t i = 0; i < servo_ids_.size(); i++)
      {
        int servo_id = servo_ids_[i];
        size_t joint_idx = servo_to_joint_map_[servo_id];
        hw_positions_[joint_idx] = ticks_to_radians(response->positions[i]);
      }
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("ST3215HardwareInterface"),
        *node_->get_clock(), 1000,
        "Read positions failed: %s", response->message.c_str());
    }
  }
  else
  {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("ST3215HardwareInterface"),
      *node_->get_clock(), 1000,
      "Read positions timeout");
  }

  // Read velocities (for servo 1 in velocity mode)
  auto vel_request = std::make_shared<srv::ReadVelocities::Request>();
  vel_request->servo_ids = servo_ids_;
  
  auto vel_future = read_velocities_client_->async_send_request(vel_request);
  
  if (rclcpp::spin_until_future_complete(node_, vel_future, 100ms) == 
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = vel_future.get();
    if (response->success && response->velocities.size() == servo_ids_.size())
    {
      for (size_t i = 0; i < servo_ids_.size(); i++)
      {
        int servo_id = servo_ids_[i];
        size_t joint_idx = servo_to_joint_map_[servo_id];
        // Convert ticks/s to rad/s (4096 ticks = 2π radians)
        hw_velocities_[joint_idx] = (static_cast<double>(response->velocities[i]) / 4096.0) * 2.0 * M_PI;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ST3215HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Determine which servos have velocity vs position command interfaces
  std::vector<size_t> velocity_servo_indices;
  std::vector<size_t> position_servo_indices;
  
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    bool has_velocity = false;
    bool has_position = false;
    
    for (const auto & cmd_interface : info_.joints[i].command_interfaces)
    {
      if (cmd_interface.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity = true;
      }
      if (cmd_interface.name == hardware_interface::HW_IF_POSITION) {
        has_position = true;
      }
    }
    
    if (has_velocity) {
      velocity_servo_indices.push_back(i);
    }
    if (has_position) {
      position_servo_indices.push_back(i);
    }
  }
  
  // Send velocity commands to servos with velocity interfaces
  if (!velocity_servo_indices.empty())
  {
    auto vel_request = std::make_shared<srv::WriteVelocities::Request>();
    
    for (size_t idx : velocity_servo_indices)
    {
      vel_request->servo_ids.push_back(servo_ids_[idx]);
      vel_request->velocities.push_back(static_cast<int>(hw_velocity_commands_[idx]));
    }
    
    auto vel_future = write_velocities_client_->async_send_request(vel_request);
    
    if (rclcpp::spin_until_future_complete(node_, vel_future, 100ms) == 
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = vel_future.get();
      if (!response->success)
      {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("ST3215HardwareInterface"),
          *node_->get_clock(), 1000,
          "Velocity write failed: %s", response->message.c_str());
      }
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("ST3215HardwareInterface"),
        *node_->get_clock(), 1000,
        "Velocity write timeout");
    }
  }
  
  // Send position commands to servos with position interfaces
  if (!position_servo_indices.empty())
  {
    auto pos_request = std::make_shared<srv::WritePositions::Request>();
    
    for (size_t idx : position_servo_indices)
    {
      pos_request->servo_ids.push_back(servo_ids_[idx]);
      
      if (!std::isnan(hw_position_commands_[idx]))
      {
        pos_request->positions.push_back(radians_to_ticks(hw_position_commands_[idx]));
      }
      else
      {
        // If command is NaN, use current position
        pos_request->positions.push_back(radians_to_ticks(hw_positions_[idx]));
      }
    }
    
    auto pos_future = write_client_->async_send_request(pos_request);
    
    if (rclcpp::spin_until_future_complete(node_, pos_future, 100ms) == 
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = pos_future.get();
      if (!response->success)
      {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("ST3215HardwareInterface"),
          *node_->get_clock(), 1000,
          "Position write failed: %s", response->message.c_str());
      }
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("ST3215HardwareInterface"),
        *node_->get_clock(), 1000,
        "Position write timeout");
    }
  }

  return hardware_interface::return_type::OK;
}

double ST3215HardwareInterface::ticks_to_radians(int ticks)
{
  return (static_cast<double>(ticks) / 4096.0) * 2.0 * M_PI;
}

int ST3215HardwareInterface::radians_to_ticks(double radians)
{
  int ticks = static_cast<int>((radians / (2.0 * M_PI)) * 4096.0);
  if (ticks < 0) ticks = 0;
  if (ticks > 4095) ticks = 4095;
  return ticks;
}

}  // namespace ugv_motor_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ugv_motor_driver::ST3215HardwareInterface, hardware_interface::SystemInterface)
