#pragma once
#include <atomic>
#include <memory>
#include <string>
#include <controller_interface/controller_interface.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include "flash_fr3_model_controller/continuous_effort_core.hpp"
#include "flash_fr3_model_controller/rollout_torque_law.hpp"

namespace flash_fr3_model_controller {
class ContinuousEffortController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  controller_interface::return_type update(const rclcpp::Time&,const rclcpp::Duration&) override;
 protected:
  // Isolated tests prescribe a monotonic clock; production uses steady_clock.
  virtual int64_t steady_clock_ns() const;
 private:
  void publish_status(double now,double age,double robot,const Joints& q,const Joints& dq);
  controller_interface::return_type fault(TorqueFault reason,double now,double age,double robot,
                                         const Joints& q,const Joints& dq);
  ContinuousEffortCore core_;RolloutTorqueLaw law_;TorqueSettings settings_;TorqueOutput output_{};
  Joints expected_{};std::string session_id_;double session_seconds_{8.};
  bool allow_activation_{false},consumed_{false},first_update_{true};
  bool task_rollout_{false};
  TorqueFault torque_fault_{TorqueFault::None};
  std::array<std::size_t,7> q_index_{},dq_index_{},command_index_{};
  std::size_t state_index_{};double last_status_{};
  int last_read_sequence_{-1},last_status_sequence_{-2},last_status_candidate_{-2};
  Phase last_status_phase_{Phase::Error};
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> model_;
  realtime_tools::RealtimeBuffer<PreparedEffortPacket> candidates_;
  std::atomic<int64_t> activated_ros_ns_{0},heartbeat_source_ns_{0},heartbeat_receipt_ns_{0},packet_source_ns_{0};
  std::atomic<bool> finish_{false},request_deactivate_{false},deactivate_sent_{false};
  std::atomic<uint64_t> invalid_packets_{0};
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr heartbeat_,finish_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr candidate_subscription_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr stop_client_;
  rclcpp::TimerBase::SharedPtr stop_timer_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> status_;
};
}  // namespace flash_fr3_model_controller
