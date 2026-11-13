#pragma once
#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <controller_interface/controller_interface.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include "flash_fr3_model_controller/model_core.hpp"

namespace flash_fr3_model_controller {
class ModelController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;
 private:
  void status(double steady, double age, const Joints& q, const Joints& dq);
  ModelCore core_;
  Joints expected_{};
  std::array<std::size_t, 7> q_index_{}, dq_index_{}, command_index_{};
  std::size_t state_index_{0};
  Plan plan_{};
  std::string plan_id_;
  bool first_update_{true};
  bool consumed_{false};
  Phase last_status_phase_{Phase::Error};
  std::atomic<bool> request_deactivate_{false}, deactivate_sent_{false};
  std::atomic<int64_t> activated_ros_ns_{0}, last_source_ns_{0}, receipt_steady_ns_{0};
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr heartbeat_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr stop_client_;
  rclcpp::TimerBase::SharedPtr stop_timer_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> status_;
  double last_status_{0.};
};
}  // namespace flash_fr3_model_controller
