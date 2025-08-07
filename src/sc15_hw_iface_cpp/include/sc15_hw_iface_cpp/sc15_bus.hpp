#pragma once
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>  // Add this
#include <std_msgs/msg/float64_multi_array.hpp>  // Add this

#include <termios.h>
#include <string>
#include <vector>

namespace sc15_hw_iface_cpp
{

class SC15Bus : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SC15Bus)

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &) override;

  hardware_interface::return_type read(
      const rclcpp::Time &, const rclcpp::Duration &) override;

  hardware_interface::return_type write(
      const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  bool open_port(const std::string & device, int baud);
  int  fd_{-1};
  std::string device_;  // Add this line
  int baud_;            // Add this line

  std::vector<int>    ids_;       // servo IDs
  std::vector<double> pos_rad_;   // last-read positions
  std::vector<double> cmd_rad_;   // desired positions

  // tick↔rad conversion
  const double tick2rad_{2.0 * M_PI / 1024.0};
  const double rad2tick_{1024.0 / (2.0 * M_PI)};

  // Add ROS 2 communication
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pos_sub_;
  
  void position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace sc15_hw_iface_cpp
