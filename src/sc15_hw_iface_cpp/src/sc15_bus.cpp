#include "sc15_hw_iface_cpp/sc15_bus.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

namespace sc15_hw_iface_cpp
{

hardware_interface::CallbackReturn SC15Bus::on_init(const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  // Get device and baud parameters
  device_ = info_.hardware_parameters["device"];
  baud_ = std::stoi(info_.hardware_parameters["baud"]);

  // Initialize vectors for joints
  for (const auto &joint : info_.joints)
  {
    ids_.push_back(std::stoi(joint.parameters.at("id")));
    pos_rad_.push_back(0.0);
    cmd_rad_.push_back(0.0);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SC15Bus::on_configure(const rclcpp_lifecycle::State &)
{
  // Note: We're not actually opening the serial port since Python bridge handles it
  RCLCPP_INFO(rclcpp::get_logger("SC15Bus"), "SC15 bus configured for %zu joints via Python bridge", ids_.size());

  // Add ROS 2 publishers/subscribers for servo bridge communication
  node_ = rclcpp::Node::make_shared("sc15_hardware_interface");

  cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/servo_commands", 10);

  pos_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/servo_positions", 10,
      std::bind(&SC15Bus::position_callback, this, std::placeholders::_1));

  RCLCPP_INFO(rclcpp::get_logger("SC15Bus"), "ROS 2 servo bridge communication initialized");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SC15Bus::on_activate(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SC15Bus::on_deactivate(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SC15Bus::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < ids_.size(); ++i)
  {
    // Use joint names from info_.joints
    std::string joint_name = info_.joints[i].name;
    state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &pos_rad_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SC15Bus::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < ids_.size(); ++i)
  {
    // Use joint names from info_.joints
    std::string joint_name = info_.joints[i].name;
    command_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &cmd_rad_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type
SC15Bus::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Spin the node to process incoming position messages
  rclcpp::spin_some(node_);
  
  RCLCPP_INFO_THROTTLE(rclcpp::get_logger("SC15Bus"), 
                       *node_->get_clock(), 2000,
                       "Read: positions updated from servo bridge");
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
SC15Bus::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  RCLCPP_INFO_THROTTLE(rclcpp::get_logger("SC15Bus"), 
                       *node_->get_clock(), 1000,
                       "Write: sending commands to servo bridge");
  
  // Publish commands to servo bridge
  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data.resize(cmd_rad_.size());
  for (size_t i = 0; i < cmd_rad_.size(); ++i) {
    msg.data[i] = cmd_rad_[i];
  }
  
  cmd_pub_->publish(msg);
  
  return hardware_interface::return_type::OK;
}

void SC15Bus::position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() == pos_rad_.size()) {
    for (size_t i = 0; i < pos_rad_.size(); ++i) {
      pos_rad_[i] = msg->data[i];
    }
  }
}

bool SC15Bus::open_port(const std::string & device, int baud)
{
  // Not used anymore since Python bridge handles serial communication
  return true;
}

} // namespace sc15_hw_iface_cpp

PLUGINLIB_EXPORT_CLASS(sc15_hw_iface_cpp::SC15Bus, hardware_interface::SystemInterface)
