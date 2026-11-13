#include "flash_fr3_pilot_controller/pilot_controller.hpp"
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.hpp>

namespace flash_fr3_pilot_controller {
namespace {
int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
}
controller_interface::InterfaceConfiguration PilotController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL};
  for (int i = 1; i <= 7; ++i) c.names.push_back("fr3_joint" + std::to_string(i) + "/position");
  return c;
}
controller_interface::InterfaceConfiguration PilotController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL};
  for (int i = 1; i <= 7; ++i) {
    c.names.push_back("fr3_joint" + std::to_string(i) + "/position");
    c.names.push_back("fr3_joint" + std::to_string(i) + "/velocity");
  }
  c.names.push_back("fr3/robot_state");
  return c;
}
controller_interface::CallbackReturn PilotController::on_init() {
  auto_declare<std::vector<double>>("expected_start_q", {});
  auto_declare<double>("joint2_delta_rad", 0.);
  auto_declare<std::string>("plan_id", "");
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn PilotController::on_configure(const rclcpp_lifecycle::State&) {
  const auto q = get_node()->get_parameter("expected_start_q").as_double_array();
  delta_ = get_node()->get_parameter("joint2_delta_rad").as_double();
  plan_id_ = get_node()->get_parameter("plan_id").as_string();
  if (q.size() != 7 || plan_id_.size() < 16 || !std::isfinite(delta_) ||
      std::abs(delta_) > .005 || std::abs(delta_) < 1e-6 ||
      (get_node()->has_parameter("use_sim_time") && get_node()->get_parameter("use_sim_time").as_bool()))
    return controller_interface::CallbackReturn::FAILURE;
  std::copy(q.begin(), q.end(), expected_.begin());
  if (!finite(expected_)) return controller_interface::CallbackReturn::FAILURE;
  heartbeat_ = get_node()->create_subscription<std_msgs::msg::Header>(
      "~/heartbeat", rclcpp::QoS(1).best_effort(), [this](std_msgs::msg::Header::ConstSharedPtr msg) {
        const int64_t activation = activated_ros_ns_.load();
        if (msg->stamp.sec < 0 || msg->stamp.nanosec >= 1000000000) return;
        const int64_t source = static_cast<int64_t>(msg->stamp.sec)*1000000000 + msg->stamp.nanosec;
        const int64_t now = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();
        const int64_t previous = last_source_ns_.load();
        if (!valid_heartbeat(source, now, activation, previous, msg->frame_id == plan_id_)) return;
        last_source_ns_.store(source);
        receipt_steady_ns_.store(steady_ns());
      });
  status_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/status", rclcpp::QoS(10));
  status_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(status_pub_);
  status_->msg_.data.resize(32, 0.);
  // Humble 2.52 logs update(ERROR) but does not automatically deactivate the
  // controller. Explicit native mode shutdown must be requested outside RT.
  stop_client_ = get_node()->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");
  stop_timer_ = get_node()->create_wall_timer(std::chrono::milliseconds(20), [this] {
    if (!request_deactivate_.load() || deactivate_sent_.load() || !stop_client_->service_is_ready()) return;
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->deactivate_controllers = {get_node()->get_name()};
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->timeout.sec = 3;
    deactivate_sent_.store(true);
    stop_client_->async_send_request(request, [this](
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture response) {
      if (!response.get()->ok) RCLCPP_ERROR(get_node()->get_logger(), "Native deactivation failed; no automatic recovery/restart.");
    });
  });
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn PilotController::on_activate(const rclcpp_lifecycle::State&) {
  // One physical trial per instance. Explicit unload/reload is required to
  // rearm, so an old asynchronous stop request cannot affect a later trial.
  if (consumed_) return controller_interface::CallbackReturn::FAILURE;
  if (command_interfaces_.size() != 7 || state_interfaces_.size() != 15)
    return controller_interface::CallbackReturn::FAILURE;
  const auto commands = command_interface_configuration().names;
  for (std::size_t j = 0; j < 7; ++j) {
    bool q_found = false, dq_found = false, c_found = false;
    const std::string joint = "fr3_joint" + std::to_string(j + 1);
    for (std::size_t i = 0; i < state_interfaces_.size(); ++i) {
      if (state_interfaces_[i].get_name() == joint + "/position") { q_index_[j] = i; q_found = true; }
      if (state_interfaces_[i].get_name() == joint + "/velocity") { dq_index_[j] = i; dq_found = true; }
    }
    for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
      if (command_interfaces_[i].get_name() == commands[j]) { command_index_[j] = i; c_found = true; }
    if (!q_found || !dq_found || !c_found) return controller_interface::CallbackReturn::FAILURE;
  }
  bool found = false;
  for (std::size_t i = 0; i < state_interfaces_.size(); ++i)
    if (state_interfaces_[i].get_name() == "fr3/robot_state") { state_index_ = i; found = true; }
  if (!found) return controller_interface::CallbackReturn::FAILURE;
  first_update_ = true;
  consumed_ = true;
  request_deactivate_.store(false);
  deactivate_sent_.store(false);
  last_source_ns_.store(0);
  receipt_steady_ns_.store(0);
  activated_ros_ns_.store(rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds());
  status_pub_->on_activate();
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn PilotController::on_deactivate(const rclcpp_lifecycle::State&) {
  activated_ros_ns_.store(0);
  status_pub_->on_deactivate();
  return controller_interface::CallbackReturn::SUCCESS;
}
void PilotController::status(double steady, double age, const Joints& q, const Joints& dq) {
  if (steady - last_status_ < .04 && core_.phase() == last_status_phase_) return;
  if (!status_->trylock()) return;
  auto& d = status_->msg_.data;
  d[0] = steady; d[1] = static_cast<int>(core_.phase());
  d[2] = static_cast<int>(core_.reason()); d[3] = age;
  for (std::size_t j = 0; j < 7; ++j) {
    d[4+j] = q[j]; d[11+j] = dq[j];
    d[18+j] = core_.reference().q[j]; d[25+j] = core_.reference().dq[j];
  }
  status_->unlockAndPublish();
  last_status_ = steady;
  last_status_phase_ = core_.phase();
}
controller_interface::return_type PilotController::update(const rclcpp::Time&, const rclcpp::Duration&) {
  const int64_t steady = steady_ns();
  const double now = steady * 1e-9;
  const int64_t receipt = receipt_steady_ns_.load();
  const double age = receipt > 0 ? std::max(0., (steady - receipt)*1e-9) : -1.;
  Joints q{}, dq{};
  for (std::size_t j = 0; j < 7; ++j) {
    q[j] = state_interfaces_[q_index_[j]].get_value();
    dq[j] = state_interfaces_[dq_index_[j]].get_value();
  }
  // Same pointer encoding as installed franka_semantic_components. This must
  // be built against the installed libfranka ABI, never a second SDK version.
  const double encoded = state_interfaces_[state_index_].get_value();
  franka::RobotState* robot = nullptr;
  static_assert(sizeof(robot) == sizeof(encoded));
  std::memcpy(&robot, &encoded, sizeof(robot));
  if (!robot) {
    request_deactivate_.store(true);
    return controller_interface::return_type::ERROR;
  }
  if (first_update_) {
    // Installed SDK exposes q_d/dq_d/ddq_d (newer docs call the raw command q_c).
    // Start at the native desired position, not at a model-predicted position.
    if (!finite(robot->dq_d) || !finite(robot->ddq_d)) {
      request_deactivate_.store(true);
      return controller_interface::return_type::ERROR;
    }
    for (std::size_t j = 0; j < 7; ++j)
      if (std::abs(robot->dq_d[j]) > .001 || std::abs(robot->ddq_d[j]) > .01) {
        request_deactivate_.store(true);
        return controller_interface::return_type::ERROR;
      }
    if (!core_.arm(robot->q_d, q, dq, expected_, delta_, robot->time.toSec(), now)) {
      status(now, age, q, dq);
      request_deactivate_.store(true);
      return controller_interface::return_type::ERROR;
    }
    first_update_ = false;
  }
  const bool valid = core_.step(robot->time.toSec(), now, age, q, dq);
  status(now, age, q, dq);
  if (!valid || core_.phase() == Phase::LatchedHold ||
      (core_.phase() == Phase::Complete && (age < 0 || age > kHeartbeatSeconds)))
    request_deactivate_.store(true);
  if (!valid) return controller_interface::return_type::ERROR;
  for (std::size_t j = 0; j < 7; ++j) command_interfaces_[command_index_[j]].set_value(core_.reference().q[j]);
  return controller_interface::return_type::OK;
}
}  // namespace flash_fr3_pilot_controller
PLUGINLIB_EXPORT_CLASS(flash_fr3_pilot_controller::PilotController, controller_interface::ControllerInterface)
