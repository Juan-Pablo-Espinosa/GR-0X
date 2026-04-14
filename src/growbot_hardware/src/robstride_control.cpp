#include "motor_ros2/motor_cfg.h"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace growbot_hardware {

using std::make_unique;
using std::stof;
using std::stoi;
using std::string;
using std::unique_ptr;
using std::vector;

class RobStrideSystem : public hardware_interface::SystemInterface {
public:
//callbacks




  // callback functions and config for motors important!!!!!
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    can_interface_ = get_param("can_interface", string("can0"));
    master_id_ = stoi(get_param("master_id", string("255")));
    default_speed_ = stof(get_param("default_speed", string("5.0")));

    const auto n = info_.joints.size();
    pos_state_.assign(n, 0.0);
    vel_state_.assign(n, 0.0);
    eff_state_.assign(n, 0.0);
    pos_cmd_.assign(n, 0.0);
    motors_.reserve(n);

    for (size_t i = 0; i < info_.joints.size(); ++i) {
      const auto &j = info_.joints[i];
      if (j.parameters.count("passive")) {
        continue;
      }
      const uint8_t motor_id = stoi(j.parameters.at("motor_id"));
      const int actuator_type =
          j.parameters.count("actuator_type")
              ? stoi(j.parameters.at("actuator_type"))
              : 0;
      motors_.emplace_back(make_unique<RobStrideMotor>(
          can_interface_, master_id_, motor_id, actuator_type));
      active_idx_.push_back(i);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }



  //state interface less important just lists the joints
  vector<hardware_interface::StateInterface>
  export_state_interfaces() override {
    vector<hardware_interface::StateInterface> stInterface;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      //constructs p v a vectors for all the joints 
      stInterface.emplace_back(info_.joints[i].name,"position", &pos_state_[i]);
      stInterface.emplace_back(info_.joints[i].name,"velocity", &vel_state_[i]);
      stInterface.emplace_back(info_.joints[i].name,"effort", &eff_state_[i]);
    }
    return stInterface;
  }
  
  
  //command processing
  vector<hardware_interface::CommandInterface>
  export_command_interfaces() override {
    vector<hardware_interface::CommandInterface> ci;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      if (info_.joints[i].parameters.count("passive")) continue;
      ci.emplace_back(info_.joints[i].name, "position", &pos_cmd_[i]);
    }
    return ci;
  }
  
  //callback on activation
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & /*prev*/) override {
    for (size_t k = 0; k < motors_.size(); ++k) {
      const size_t i = active_idx_[k];
      auto [p, v, t, temp] = motors_[k]->enable_motor();
      pos_state_[i] = p;
      vel_state_[i] = v;
      eff_state_[i] = t;
      pos_cmd_[i] = p;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }
//callback on deactivation
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & /*prev*/) override {
    for (auto &m : motors_)
      m->Disenable_Motor(0);
    return hardware_interface::CallbackReturn::SUCCESS;
  }






  //Actually interesting parts !!!!!!!!!!!!!!!!!!
  hardware_interface::return_type
  read(const rclcpp::Time &, const rclcpp::Duration &) override {
    return hardware_interface::return_type::OK;
  }
 
  hardware_interface::return_type
  write(const rclcpp::Time &, const rclcpp::Duration &) override {
    for (size_t k = 0; k < motors_.size(); ++k) {
      const size_t i = active_idx_[k];
      auto [p, v, t, temp] = motors_[k]->RobStrite_Motor_PosCSP_control(
          default_speed_, static_cast<float>(pos_cmd_[i]));
      pos_state_[i] = p;
      vel_state_[i] = v;
      eff_state_[i] = t;
    }
    //signals tick complete
    return hardware_interface::return_type::OK;
  }

private:
  template <typename T>
  T get_param(const string &key, const T &fallback) const {
    auto it = info_.hardware_parameters.find(key);
    return it == info_.hardware_parameters.end() ? fallback : T(it->second);
  }

  string can_interface_;
  uint8_t master_id_ = 0xFF;
  float default_speed_ = 5.0f;

  vector<unique_ptr<RobStrideMotor>> motors_;
  vector<size_t> active_idx_;  // motors_[k] drives info_.joints[active_idx_[k]]
  vector<double> pos_state_, vel_state_, eff_state_;
  vector<double> pos_cmd_;
};

} // namespace growbot_hardware

PLUGINLIB_EXPORT_CLASS(growbot_hardware::RobStrideSystem,
                      hardware_interface::SystemInterface)
