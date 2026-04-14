#include "hardware_interface/system_interface.hpp"
//NOT USED CURRENTLy
//I did not write this btw just a clear example of hardware
class MyRobot : public hardware_interface::SystemInterface
{
public:
  // Lifecycle.
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // Interface exports.
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Real-time loop.
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // One double per joint per interface type.
  std::vector<double> hw_positions_, hw_velocities_;
  std::vector<double> hw_commands_;
};