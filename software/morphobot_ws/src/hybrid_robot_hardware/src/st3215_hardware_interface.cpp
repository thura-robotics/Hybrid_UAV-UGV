#include "hybrid_robot_hardware/st3215_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace hybrid_robot_hardware
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

  // Resize state and command vectors
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Verify joint configuration
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ST3215HardwareInterface"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ST3215HardwareInterface"),
        "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
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
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ST3215HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ST3215HardwareInterface"), "Activating hardware interface...");

  // Read initial positions
  auto request = std::make_shared<srv::ReadPositions::Request>();
  request->servo_ids = servo_ids_;
  
  auto result = read_client_->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(node_, result, 5s) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result.get();
    if (response->success && response->positions.size() == servo_ids_.size())
    {
      for (size_t i = 0; i < servo_ids_.size(); i++)
      {
        hw_positions_[i] = ticks_to_radians(response->positions[i]);
        hw_position_commands_[i] = hw_positions_[i];
        hw_velocities_[i] = 0.0;
        hw_efforts_[i] = 0.0;
        
        RCLCPP_INFO(
          rclcpp::get_logger("ST3215HardwareInterface"),
          "Servo %d initial position: %.3f rad", servo_ids_[i], hw_positions_[i]);
      }
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("ST3215HardwareInterface"),
                   "Failed to read initial positions: %s", response->message.c_str());
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
  // Create request
  auto request = std::make_shared<srv::ReadPositions::Request>();
  request->servo_ids = servo_ids_;
  
  // Call service (async)
  auto result_future = read_client_->async_send_request(request);
  
  // Wait for response with timeout
  if (rclcpp::spin_until_future_complete(node_, result_future, 100ms) == 
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result_future.get();
    if (response->success && response->positions.size() == servo_ids_.size())
    {
      for (size_t i = 0; i < servo_ids_.size(); i++)
      {
        hw_positions_[i] = ticks_to_radians(response->positions[i]);
      }
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("ST3215HardwareInterface"),
        *node_->get_clock(), 1000,
        "Read failed: %s", response->message.c_str());
    }
  }
  else
  {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("ST3215HardwareInterface"),
      *node_->get_clock(), 1000,
      "Read timeout");
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ST3215HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Create request
  auto request = std::make_shared<srv::WritePositions::Request>();
  request->servo_ids = servo_ids_;
  request->positions.clear();
  
  for (size_t i = 0; i < servo_ids_.size(); i++)
  {
    if (!std::isnan(hw_position_commands_[i]))
    {
      request->positions.push_back(radians_to_ticks(hw_position_commands_[i]));
    }
    else
    {
      // If command is NaN, use current position
      request->positions.push_back(radians_to_ticks(hw_positions_[i]));
    }
  }
  
  // Call service (async)
  auto result_future = write_client_->async_send_request(request);
  
  // Wait for response with timeout
  if (rclcpp::spin_until_future_complete(node_, result_future, 100ms) == 
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result_future.get();
    if (!response->success)
    {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("ST3215HardwareInterface"),
        *node_->get_clock(), 1000,
        "Write failed: %s", response->message.c_str());
    }
  }
  else
  {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("ST3215HardwareInterface"),
      *node_->get_clock(), 1000,
      "Write timeout");
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

}  // namespace hybrid_robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  hybrid_robot_hardware::ST3215HardwareInterface, hardware_interface::SystemInterface)
