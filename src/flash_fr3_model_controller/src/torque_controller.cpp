#include "flash_fr3_model_controller/torque_controller.hpp"
#include <chrono>
#include <cstring>
#include <exception>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.hpp>

namespace flash_fr3_model_controller {
namespace {
int64_t torque_steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
template<class T> T* decode_pointer(double encoded) {
  T* pointer=nullptr;
  static_assert(sizeof(pointer)==sizeof(encoded));
  std::memcpy(&pointer,&encoded,sizeof(pointer));
  return pointer;
}
}
controller_interface::InterfaceConfiguration TorqueController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration out{controller_interface::interface_configuration_type::INDIVIDUAL};
  for (int i=1;i<=7;++i) out.names.push_back("fr3_joint"+std::to_string(i)+"/effort");
  return out;
}
controller_interface::InterfaceConfiguration TorqueController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration out{controller_interface::interface_configuration_type::INDIVIDUAL};
  for (int i=1;i<=7;++i) {
    out.names.push_back("fr3_joint"+std::to_string(i)+"/position");
    out.names.push_back("fr3_joint"+std::to_string(i)+"/velocity");
  }
  out.names.push_back("fr3/robot_state");
  out.names.push_back("fr3/robot_model");
  return out;
}
controller_interface::CallbackReturn TorqueController::on_init() {
  for (const auto& name:{"expected_start_q","segment_durations","segment_coefficients",
                         "k_gains","d_gains","torque_limits_nm","torque_rates_nm_s"})
    auto_declare<std::vector<double>>(name,{});
  auto_declare<std::string>("plan_id","");
  auto_declare<bool>("allow_activation",false);
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn TorqueController::on_configure(const rclcpp_lifecycle::State&) {
  const auto q=get_node()->get_parameter("expected_start_q").as_double_array();
  const auto durations=get_node()->get_parameter("segment_durations").as_double_array();
  const auto coefficients=get_node()->get_parameter("segment_coefficients").as_double_array();
  plan_id_=get_node()->get_parameter("plan_id").as_string();
  allow_activation_=get_node()->get_parameter("allow_activation").as_bool();
  if (q.size()!=7||durations.size()!=2||coefficients.size()!=98||plan_id_.size()<16||
      (get_node()->has_parameter("use_sim_time")&&get_node()->get_parameter("use_sim_time").as_bool()))
    return controller_interface::CallbackReturn::FAILURE;
  Joints* fields[]{&settings_.kp,&settings_.kd,&settings_.magnitude,&settings_.rate};
  const char* names[]{"k_gains","d_gains","torque_limits_nm","torque_rates_nm_s"};
  for (int k=0;k<4;++k) {
    const auto values=get_node()->get_parameter(names[k]).as_double_array();
    if (values.size()!=7) return controller_interface::CallbackReturn::FAILURE;
    std::copy(values.begin(),values.end(),fields[k]->begin());
  }
  std::copy(q.begin(),q.end(),expected_.begin());
  for (std::size_t s=0;s<2;++s) {
    plan_[s].duration=durations[s];
    for (std::size_t k=0;k<7;++k) for (std::size_t j=0;j<7;++j)
      plan_[s].c[k][j]=coefficients[s*49+k*7+j];
  }
  if (!law_.configure(settings_)||!validate_torque_commissioning_plan(plan_,expected_))
    return controller_interface::CallbackReturn::FAILURE;
  heartbeat_=get_node()->create_subscription<std_msgs::msg::Header>(
      "~/heartbeat",rclcpp::QoS(1).best_effort(),[this](std_msgs::msg::Header::ConstSharedPtr message) {
        if (message->stamp.sec<0||message->stamp.nanosec>=1000000000) return;
        const auto source=static_cast<int64_t>(message->stamp.sec)*1000000000+message->stamp.nanosec;
        const auto now=rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();
        if (!valid_heartbeat(source,now,activated_ros_ns_.load(),last_source_ns_.load(),message->frame_id==plan_id_)) return;
        last_source_ns_.store(source);receipt_steady_ns_.store(torque_steady_ns());
      });
  status_pub_=get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/status",rclcpp::QoS(10));
  status_=std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(status_pub_);
  status_->msg_.data.resize(54,0.);
  // Separate additive telemetry: keep the existing 54-field status ABI and
  // the control law unchanged. All storage is allocated outside update().
  sdk_status_pub_=get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/sdk_status",rclcpp::QoS(10));
  sdk_status_=std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(sdk_status_pub_);
  sdk_status_->msg_.data.resize(66,0.);
  model_=std::make_unique<franka_semantic_components::FrankaRobotModel>("fr3/robot_model","fr3/robot_state");
  stop_client_=get_node()->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
  stop_timer_=get_node()->create_wall_timer(std::chrono::milliseconds(20),[this] {
    if (!request_deactivate_.load()||deactivate_sent_.load()||!stop_client_->service_is_ready()) return;
    auto request=std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->deactivate_controllers={get_node()->get_name()};
    request->strictness=controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->timeout.sec=3;
    deactivate_sent_.store(true);
    stop_client_->async_send_request(request,[this](
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
      if (!future.get()->ok) RCLCPP_ERROR(get_node()->get_logger(),"Torque controller deactivation failed; no automatic restart.");
    });
  });
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn TorqueController::on_activate(const rclcpp_lifecycle::State&) {
  if (!allow_activation_||consumed_||command_interfaces_.size()!=7||state_interfaces_.size()!=16)
    return controller_interface::CallbackReturn::FAILURE;
  const auto required=command_interface_configuration().names;
  std::size_t model_index=0;
  bool found_state=false,found_model=false;
  for (std::size_t j=0;j<7;++j) {
    bool a=false,b=false,c=false;
    const auto joint="fr3_joint"+std::to_string(j+1);
    for (std::size_t i=0;i<state_interfaces_.size();++i) {
      if (state_interfaces_[i].get_name()==joint+"/position") {q_index_[j]=i;a=true;}
      if (state_interfaces_[i].get_name()==joint+"/velocity") {dq_index_[j]=i;b=true;}
    }
    for (std::size_t i=0;i<command_interfaces_.size();++i)
      if (command_interfaces_[i].get_name()==required[j]) {command_index_[j]=i;c=true;}
    if (!a||!b||!c) return controller_interface::CallbackReturn::FAILURE;
  }
  for (std::size_t i=0;i<state_interfaces_.size();++i) {
    if (state_interfaces_[i].get_name()=="fr3/robot_state") {state_index_=i;found_state=true;}
    if (state_interfaces_[i].get_name()=="fr3/robot_model") {model_index=i;found_model=true;}
  }
  if (!found_state||!found_model||
      !decode_pointer<franka::RobotState>(state_interfaces_[state_index_].get_value())||
      !decode_pointer<franka_hardware::Model>(state_interfaces_[model_index].get_value()))
    return controller_interface::CallbackReturn::FAILURE;
  // The semantic component is rebuilt on every configuration, so its cached
  // pointers cannot leak across controller-manager hardware sessions.
  model_->assign_loaned_state_interfaces(state_interfaces_);
  consumed_=true;first_update_=true;output_={};torque_fault_=TorqueFault::None;
  previous_written_command_={};previous_written_valid_=false;
  last_sdk_status_=0.;last_sdk_phase_=Phase::Error;
  request_deactivate_.store(false);deactivate_sent_.store(false);
  last_source_ns_.store(0);receipt_steady_ns_.store(0);
  activated_ros_ns_.store(rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds());
  status_pub_->on_activate();
  sdk_status_pub_->on_activate();
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn TorqueController::on_deactivate(const rclcpp_lifecycle::State&) {
  activated_ros_ns_.store(0);model_->release_interfaces();status_pub_->on_deactivate();
  sdk_status_pub_->on_deactivate();
  return controller_interface::CallbackReturn::SUCCESS;
}
void TorqueController::publish_status(double now,double age,const Joints& q,const Joints& dq) {
  const auto phase=torque_fault_==TorqueFault::None?reference_.phase():Phase::Error;
  if (now-last_status_<.04&&phase==last_phase_) return;
  if (!status_->trylock()) return;
  auto& data=status_->msg_.data;
  data[0]=now;data[1]=static_cast<int>(phase);data[2]=static_cast<int>(reference_.reason());data[3]=age;
  for (std::size_t j=0;j<7;++j) {
    data[4+j]=q[j];data[11+j]=dq[j];data[18+j]=reference_.reference().q[j];
    data[25+j]=reference_.reference().dq[j];data[32+j]=output_.raw[j];
    data[39+j]=output_.command[j];data[46+j]=output_.limited[j]?1.:0.;
  }
  data[53]=static_cast<int>(torque_fault_);
  status_->unlockAndPublish();last_status_=now;last_phase_=phase;
}
void TorqueController::publish_sdk_status(double now,double period,const franka::RobotState& robot,
                                         const Joints& coriolis) {
  const auto phase=reference_.phase();
  if (now-last_sdk_status_<.04&&phase==last_sdk_phase_) return;
  // Never wait/retry in the control thread; missed telemetry is not a control
  // input. SDK tau_J_d is the previously accepted torque, not measured tau_J.
  if (!sdk_status_->trylock()) return;
  auto& data=sdk_status_->msg_.data;
  data[0]=1.;data[1]=now;data[2]=robot.time.toSec();data[3]=static_cast<int>(phase);
  data[4]=previous_written_valid_?1.:0.;data[5]=period;
  for (std::size_t j=0;j<7;++j) {
    data[6+j]=robot.tau_J_d[j];data[13+j]=previous_written_command_[j];
    data[20+j]=output_.command[j];data[27+j]=robot.tau_J[j];
    data[34+j]=robot.tau_ext_hat_filtered[j];data[41+j]=coriolis[j];
    data[48+j]=robot.q[j];data[55+j]=robot.dq[j];
  }
  data[62]=robot.m_ee;data[63]=robot.m_load;data[64]=robot.m_total;
  data[65]=robot.control_command_success_rate;
  sdk_status_->unlockAndPublish();last_sdk_status_=now;last_sdk_phase_=phase;
}
controller_interface::return_type TorqueController::fault(TorqueFault reason,double now,double age,
                                                          const Joints& q,const Joints& dq) {
  if (torque_fault_==TorqueFault::None) torque_fault_=reason;
  request_deactivate_.store(true);publish_status(now,age,q,dq);
  return controller_interface::return_type::ERROR;
}
controller_interface::return_type TorqueController::update(const rclcpp::Time&,const rclcpp::Duration&) {
  const auto steady=torque_steady_ns();const double now=steady*1e-9;
  const auto receipt=receipt_steady_ns_.load();const double age=receipt>0?std::max(0.,(steady-receipt)*1e-9):-1.;
  Joints q{},dq{};
  for (std::size_t j=0;j<7;++j) {q[j]=state_interfaces_[q_index_[j]].get_value();dq[j]=state_interfaces_[dq_index_[j]].get_value();}
  if (torque_fault_!=TorqueFault::None) return fault(torque_fault_,now,age,q,dq);
  auto* robot=decode_pointer<franka::RobotState>(state_interfaces_[state_index_].get_value());
  if (!robot) return fault(TorqueFault::Model,now,age,q,dq);
  const auto robot_time=robot->time.toSec();
  const bool first=first_update_;
  if (first) {
    // An effort session starts from measured q. q_d can be left over from a
    // prior internal position-control session and is not our torque reference.
    origin_=q;
    if (!validate_torque_commissioning_plan(plan_,origin_)||
        !reference_.arm(q,q,dq,expected_,plan_,robot_time,now))
      return fault(TorqueFault::ReferenceOrState,now,age,q,dq);
    previous_robot_time_=robot_time;first_update_=false;
  }
  const auto period=first?.001:robot_time-previous_robot_time_;
  previous_robot_time_=robot_time;
  if (!reference_.step(robot_time,now,age,q,dq)) return fault(TorqueFault::ReferenceOrState,now,age,q,dq);
  for (std::size_t j=0;j<7;++j)
    if (std::abs(q[j]-origin_[j])>.004) return fault(TorqueFault::ReferenceOrState,now,age,q,dq);
  Joints coriolis{};
  try {coriolis=model_->getCoriolisForceVector();}
  catch (const std::exception&) {return fault(TorqueFault::Model,now,age,q,dq);}
  if (!law_.evaluate(reference_.reference(),q,dq,coriolis,robot->tau_J_d,period,output_))
    return fault(law_.fault(),now,age,q,dq);
  // All seven commands are written only after every check passed.
  for (std::size_t j=0;j<7;++j) command_interfaces_[command_index_[j]].set_value(output_.command[j]);
  publish_status(now,age,q,dq);
  publish_sdk_status(now,period,*robot,coriolis);
  previous_written_command_=output_.command;previous_written_valid_=true;
  if (reference_.phase()==Phase::LatchedHold||reference_.phase()==Phase::Complete)
    request_deactivate_.store(true);
  return controller_interface::return_type::OK;
}
}  // namespace flash_fr3_model_controller
PLUGINLIB_EXPORT_CLASS(flash_fr3_model_controller::TorqueController,controller_interface::ControllerInterface)
