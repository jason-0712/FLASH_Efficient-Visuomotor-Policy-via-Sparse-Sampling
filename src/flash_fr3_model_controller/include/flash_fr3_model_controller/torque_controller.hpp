#pragma once
#include <atomic>
#include <memory>
#include <string>
#include <controller_interface/controller_interface.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <franka/robot_state.h>
#include "flash_fr3_model_controller/torque_core.hpp"

namespace flash_fr3_model_controller {
class TorqueController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;
 private:
  void publish_status(double steady,double age,const Joints& q,const Joints& dq);
  void publish_sdk_status(double steady,double period,const franka::RobotState& robot,
                          const Joints& coriolis);
  controller_interface::return_type fault(TorqueFault reason,double now,double age,
                                         const Joints& q,const Joints& dq);
  TaskCore reference_;
  TorqueLaw law_;
  TorqueOutput output_{};
  TorqueSettings settings_{};
  Plan plan_{};
  Joints expected_{}, origin_{};
  Joints previous_written_command_{};
  bool previous_written_valid_{false};
  std::array<std::size_t,7> q_index_{},dq_index_{},command_index_{};
  std::size_t state_index_{0};
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> model_;
  std::string plan_id_;
  bool allow_activation_{false}, consumed_{false}, first_update_{true};
  TorqueFault torque_fault_{TorqueFault::None};
  Phase last_phase_{Phase::Error};
  double previous_robot_time_{0.}, last_status_{0.};
  double last_sdk_status_{0.};
  Phase last_sdk_phase_{Phase::Error};
  std::atomic<bool> request_deactivate_{false}, deactivate_sent_{false};
  std::atomic<int64_t> activated_ros_ns_{0}, last_source_ns_{0}, receipt_steady_ns_{0};
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr heartbeat_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr stop_client_;
  rclcpp::TimerBase::SharedPtr stop_timer_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> status_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr sdk_status_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> sdk_status_;
};
}  // namespace flash_fr3_model_controller
