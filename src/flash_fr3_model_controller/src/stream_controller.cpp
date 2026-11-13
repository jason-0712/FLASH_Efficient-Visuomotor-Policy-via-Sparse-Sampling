#include "flash_fr3_model_controller/stream_controller.hpp"
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.hpp>

namespace flash_fr3_model_controller {
namespace {
int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
}
controller_interface::InterfaceConfiguration StreamController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL};
  for (int i = 1; i <= 7; ++i) c.names.push_back("fr3_joint" + std::to_string(i) + "/position");
  return c;
}
controller_interface::InterfaceConfiguration StreamController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL};
  for (int i = 1; i <= 7; ++i) {
    c.names.push_back("fr3_joint" + std::to_string(i) + "/position");
    c.names.push_back("fr3_joint" + std::to_string(i) + "/velocity");
  }
  c.names.push_back("fr3/robot_state");
  return c;
}
controller_interface::CallbackReturn StreamController::on_init() {
  auto_declare<std::vector<double>>("expected_start_q", {});
  auto_declare<std::vector<double>>("segment_durations", {});
  auto_declare<std::vector<double>>("segment_coefficients", {});
  auto_declare<std::string>("plan_id", "");
  auto_declare<int>("max_replacements", 1);
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn StreamController::on_configure(const rclcpp_lifecycle::State&) {
  const auto q = get_node()->get_parameter("expected_start_q").as_double_array();
  const auto durations = get_node()->get_parameter("segment_durations").as_double_array();
  const auto coefficients = get_node()->get_parameter("segment_coefficients").as_double_array();
  plan_id_ = get_node()->get_parameter("plan_id").as_string();
  const auto replacements = get_node()->get_parameter("max_replacements").as_int();
  if (q.size() != 7 || plan_id_.size() < 16 || durations.size() != 2 || coefficients.size() != 98 ||
      replacements < 1 || replacements > 2 ||
      (get_node()->has_parameter("use_sim_time") && get_node()->get_parameter("use_sim_time").as_bool()))
    return controller_interface::CallbackReturn::FAILURE;
  max_replacements_ = static_cast<int>(replacements);
  std::copy(q.begin(), q.end(), expected_.begin());
  for (std::size_t segment = 0; segment < 2; ++segment) {
    plan_[segment].duration = durations[segment];
    for (std::size_t k = 0; k < 7; ++k)
      for (std::size_t j = 0; j < 7; ++j)
        plan_[segment].c[k][j] = coefficients[segment*49+k*7+j];
  }
  if (!finite(expected_) || !validate_stream_plan(plan_,{})) return controller_interface::CallbackReturn::FAILURE;
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
  status_->msg_.data.resize(53, 0.);
  candidates_.writeFromNonRT(StreamPacket{});
  candidate_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/candidate", rclcpp::QoS(1).reliable(), [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        if (msg->data.size()!=104 || msg->layout.dim.size()!=1 ||
            msg->layout.dim[0].label!=plan_id_ || msg->layout.data_offset!=0 ||
            msg->layout.dim[0].size!=104 || msg->layout.dim[0].stride!=104) return;
        for (double v:msg->data) if (!std::isfinite(v)) return;
        const auto& d=msg->data;
        if (d[0]!=1. || d[1]<1. || d[1]>max_replacements_ || std::floor(d[1])!=d[1] ||
            d[3]<0. || d[3]>9e9) return;
        const int64_t source=static_cast<int64_t>(d[3]*1e9);
        const auto now=rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();
        if (!valid_heartbeat(source,now,activated_ros_ns_.load(),last_packet_source_ns_.load(),true)) return;
        StreamPacket packet;packet.sequence=static_cast<int>(d[1]);packet.handoff_robot_time=d[2];
        for (std::size_t i=0;i<2;++i) {
          packet.plan[i].duration=d[4+i];
          for (std::size_t k=0;k<7;++k) for (std::size_t j=0;j<7;++j)
            packet.plan[i].c[k][j]=d[6+i*49+k*7+j];
        }
        last_packet_source_ns_.store(source);
        candidates_.writeFromNonRT(packet);
      });
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
controller_interface::CallbackReturn StreamController::on_activate(const rclcpp_lifecycle::State&) {
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
  last_packet_source_ns_.store(0);
  receipt_steady_ns_.store(0);
  activated_ros_ns_.store(rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds());
  status_pub_->on_activate();
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn StreamController::on_deactivate(const rclcpp_lifecycle::State&) {
  activated_ros_ns_.store(0);
  status_pub_->on_deactivate();
  return controller_interface::CallbackReturn::SUCCESS;
}
void StreamController::status(double steady, double age, double robot_time, const Joints& q, const Joints& dq) {
  if (steady-last_status_<.01 && core_.phase()==last_status_phase_ &&
      core_.sequence()==last_status_sequence_ && core_.last_candidate()==last_status_candidate_) return;
  if (!status_->trylock()) return;
  auto& d = status_->msg_.data;
  d[0] = steady; d[1] = static_cast<int>(core_.phase());
  d[2] = static_cast<int>(core_.reason()); d[3] = age;
  for (std::size_t j = 0; j < 7; ++j) {
    d[4+j] = q[j]; d[11+j] = dq[j];
    d[18+j] = core_.reference().q[j]; d[25+j] = core_.reference().dq[j];
    d[32+j] = core_.reference().ddq[j]; d[46+j] = core_.origin()[j];
  }
  d[39]=robot_time;d[40]=core_.prefix_start();d[41]=core_.prefix_end();
  d[42]=core_.sequence();d[43]=core_.pending_sequence();
  d[44]=core_.last_candidate();d[45]=static_cast<int>(core_.rejection());
  status_->unlockAndPublish();
  last_status_ = steady;
  last_status_phase_ = core_.phase();
  last_status_sequence_=core_.sequence();last_status_candidate_=core_.last_candidate();
}
controller_interface::return_type StreamController::update(const rclcpp::Time&, const rclcpp::Duration&) {
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
    if (!core_.arm(robot->q_d, q, dq, expected_, plan_, robot->time.toSec(), now,max_replacements_)) {
      status(now, age, robot->time.toSec(), q, dq);
      request_deactivate_.store(true);
      return controller_interface::return_type::ERROR;
    }
    first_update_ = false;
  }
  const bool valid = core_.step(robot->time.toSec(), now, age, q, dq);
  if (valid) core_.stage(*candidates_.readFromRT(),robot->time.toSec());
  status(now, age, robot->time.toSec(), q, dq);
  if (!valid || core_.phase() == Phase::LatchedHold || core_.phase() == Phase::Complete)
    request_deactivate_.store(true);
  if (!valid) return controller_interface::return_type::ERROR;
  for (std::size_t j = 0; j < 7; ++j) command_interfaces_[command_index_[j]].set_value(core_.reference().q[j]);
  return controller_interface::return_type::OK;
}
}  // namespace flash_fr3_model_controller
PLUGINLIB_EXPORT_CLASS(flash_fr3_model_controller::StreamController, controller_interface::ControllerInterface)
