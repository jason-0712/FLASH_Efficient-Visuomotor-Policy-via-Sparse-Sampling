#include "flash_fr3_model_controller/continuous_effort_controller.hpp"
#include <chrono>
#include <cstring>
#include <exception>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.hpp>

namespace flash_fr3_model_controller {
namespace {
int64_t continuous_steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
template<class T> T* continuous_pointer(double encoded) {
  T* pointer=nullptr;static_assert(sizeof(pointer)==sizeof(encoded));
  std::memcpy(&pointer,&encoded,sizeof(pointer));return pointer;
}
}
int64_t ContinuousEffortController::steady_clock_ns() const {return continuous_steady_ns();}
controller_interface::InterfaceConfiguration ContinuousEffortController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration out{controller_interface::interface_configuration_type::INDIVIDUAL};
  for(int j=1;j<=7;++j) out.names.push_back("fr3_joint"+std::to_string(j)+"/effort");
  return out;
}
controller_interface::InterfaceConfiguration ContinuousEffortController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration out{controller_interface::interface_configuration_type::INDIVIDUAL};
  for(int j=1;j<=7;++j) {
    out.names.push_back("fr3_joint"+std::to_string(j)+"/position");
    out.names.push_back("fr3_joint"+std::to_string(j)+"/velocity");
  }
  out.names.push_back("fr3/robot_state");out.names.push_back("fr3/robot_model");return out;
}
controller_interface::CallbackReturn ContinuousEffortController::on_init() {
  for(const auto& name:{"expected_start_q","k_gains","d_gains","torque_limits_nm","torque_rates_nm_s"})
    auto_declare<std::vector<double>>(name,{});
  auto_declare<std::string>("session_id","");auto_declare<bool>("allow_activation",false);
  auto_declare<std::string>("execution_profile","commissioning");
  auto_declare<double>("maximum_session_seconds",8.);return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn ContinuousEffortController::on_configure(const rclcpp_lifecycle::State&) {
  const auto q=get_node()->get_parameter("expected_start_q").as_double_array();
  session_id_=get_node()->get_parameter("session_id").as_string();
  session_seconds_=get_node()->get_parameter("maximum_session_seconds").as_double();
  allow_activation_=get_node()->get_parameter("allow_activation").as_bool();
  const auto profile=get_node()->get_parameter("execution_profile").as_string();
  if(profile!="commissioning"&&profile!="raw_policy_rollout") return controller_interface::CallbackReturn::FAILURE;
  task_rollout_=profile=="raw_policy_rollout";
  if(q.size()!=7||session_id_.size()<16||session_id_.size()>128||!std::isfinite(session_seconds_)||
     session_seconds_<1.||
     (get_node()->has_parameter("use_sim_time")&&get_node()->get_parameter("use_sim_time").as_bool()))
    return controller_interface::CallbackReturn::FAILURE;
  std::copy(q.begin(),q.end(),expected_.begin());if(!finite(expected_)) return controller_interface::CallbackReturn::FAILURE;
  Joints* fields[]{&settings_.kp,&settings_.kd,&settings_.magnitude,&settings_.rate};
  const char* names[]{"k_gains","d_gains","torque_limits_nm","torque_rates_nm_s"};
  for(int k=0;k<4;++k) {
    const auto values=get_node()->get_parameter(names[k]).as_double_array();
    if(values.size()!=7) return controller_interface::CallbackReturn::FAILURE;
    std::copy(values.begin(),values.end(),fields[k]->begin());
  }
  if(!law_.configure(settings_,task_rollout_)) return controller_interface::CallbackReturn::FAILURE;
  heartbeat_=get_node()->create_subscription<std_msgs::msg::Header>("~/heartbeat",rclcpp::QoS(1).best_effort(),
    [this](std_msgs::msg::Header::ConstSharedPtr msg) {
      if(msg->stamp.sec<0||msg->stamp.nanosec>=1000000000) return;
      const int64_t source=static_cast<int64_t>(msg->stamp.sec)*1000000000+msg->stamp.nanosec;
      if(!valid_heartbeat(source,rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds(),
                         activated_ros_ns_.load(),heartbeat_source_ns_.load(),msg->frame_id==session_id_)) return;
      heartbeat_source_ns_.store(source);heartbeat_receipt_ns_.store(steady_clock_ns());
    });
  finish_subscription_=get_node()->create_subscription<std_msgs::msg::Header>("~/finish",rclcpp::QoS(1).reliable(),
    [this](std_msgs::msg::Header::ConstSharedPtr msg) {
      if(msg->stamp.sec<0||msg->stamp.nanosec>=1000000000) return;
      const int64_t source=static_cast<int64_t>(msg->stamp.sec)*1000000000+msg->stamp.nanosec;
      if(valid_heartbeat(source,rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds(),
                         activated_ros_ns_.load(),0,msg->frame_id==session_id_)) finish_.store(true);
    });
  candidates_.writeFromNonRT(PreparedEffortPacket{});
  candidate_subscription_=get_node()->create_subscription<std_msgs::msg::Float64MultiArray>("~/candidate",rclcpp::QoS(1).reliable(),
    [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
      auto invalid=[this]{invalid_packets_.fetch_add(1);};
      if(msg->layout.dim.size()!=1||msg->layout.dim[0].label!=session_id_||msg->layout.data_offset!=0||
         msg->layout.dim[0].size!=msg->data.size()||msg->layout.dim[0].stride!=msg->data.size()) {invalid();return;}
      PhysicalPacket raw;
      if(!decode_effort_packet(msg->data.data(),msg->data.size(),raw)) {invalid();return;}
      const int64_t source=static_cast<int64_t>(msg->data[2]*1e9);
      if(!valid_heartbeat(source,rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds(),activated_ros_ns_.load(),
                          packet_source_ns_.load(),true)) {invalid();return;}
      // The RT core checks these same observation-age and 20 ms lead limits.
      // Let it report a sequence-tagged Clock rejection. Silently dropping a
      // well-formed late candidate here leaves the client awaiting its result.
      PreparedEffortPacket packet;
      if(!prepare_effort_packet(raw,packet,task_rollout_)) {invalid();return;}
      packet_source_ns_.store(source);candidates_.writeFromNonRT(packet);
    });
  model_=std::make_unique<franka_semantic_components::FrankaRobotModel>("fr3/robot_model","fr3/robot_state");
  status_pub_=get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/status",rclcpp::QoS(10));
  status_=std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(status_pub_);
  status_->msg_.data.resize(70,0.);
  stop_client_=get_node()->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
  stop_timer_=get_node()->create_wall_timer(std::chrono::milliseconds(20),[this] {
    if(!request_deactivate_.load()||deactivate_sent_.load()||!stop_client_->service_is_ready()) return;
    auto request=std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->deactivate_controllers={get_node()->get_name()};
    request->strictness=controller_manager_msgs::srv::SwitchController::Request::STRICT;request->timeout.sec=3;
    deactivate_sent_.store(true);
    stop_client_->async_send_request(request,[this](
      rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture response) {
        if(!response.get()->ok) RCLCPP_ERROR(get_node()->get_logger(),"Continuous effort deactivation rejected; no automatic restart.");
      });
  });
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn ContinuousEffortController::on_activate(const rclcpp_lifecycle::State&) {
  if(!allow_activation_||consumed_||command_interfaces_.size()!=7||state_interfaces_.size()!=16)
    return controller_interface::CallbackReturn::FAILURE;
  const auto required=command_interface_configuration().names;
  for(std::size_t j=0;j<7;++j) {
    bool a=false,b=false,c=false;const auto joint="fr3_joint"+std::to_string(j+1);
    for(std::size_t i=0;i<state_interfaces_.size();++i) {
      if(state_interfaces_[i].get_name()==joint+"/position") {q_index_[j]=i;a=true;}
      if(state_interfaces_[i].get_name()==joint+"/velocity") {dq_index_[j]=i;b=true;}
    }
    for(std::size_t i=0;i<command_interfaces_.size();++i)
      if(command_interfaces_[i].get_name()==required[j]) {command_index_[j]=i;c=true;}
    if(!a||!b||!c) return controller_interface::CallbackReturn::FAILURE;
  }
  bool a=false,b=false;std::size_t model_index=0;
  for(std::size_t i=0;i<state_interfaces_.size();++i) {
    if(state_interfaces_[i].get_name()=="fr3/robot_state") {state_index_=i;a=true;}
    if(state_interfaces_[i].get_name()=="fr3/robot_model") {model_index=i;b=true;}
  }
  if(!a||!b||!continuous_pointer<franka::RobotState>(state_interfaces_[state_index_].get_value())||
     !continuous_pointer<franka_hardware::Model>(state_interfaces_[model_index].get_value()))
    return controller_interface::CallbackReturn::FAILURE;
  model_->assign_loaned_state_interfaces(state_interfaces_);
  consumed_=true;first_update_=true;output_={};core_=ContinuousEffortCore{};
  torque_fault_=TorqueFault::None;last_read_sequence_=-1;
  finish_.store(false);request_deactivate_.store(false);deactivate_sent_.store(false);
  heartbeat_source_ns_.store(0);heartbeat_receipt_ns_.store(0);packet_source_ns_.store(0);
  activated_ros_ns_.store(rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds());
  status_pub_->on_activate();return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn ContinuousEffortController::on_deactivate(const rclcpp_lifecycle::State&) {
  activated_ros_ns_.store(0);model_->release_interfaces();status_pub_->on_deactivate();
  return controller_interface::CallbackReturn::SUCCESS;
}
void ContinuousEffortController::publish_status(double now,double age,double robot,const Joints& q,const Joints& dq) {
  const auto phase=torque_fault_==TorqueFault::None?core_.phase():Phase::Error;
  if(now-last_status_<.04&&phase==last_status_phase_&&core_.sequence()==last_status_sequence_&&
     core_.last_candidate()==last_status_candidate_) return;
  if(!status_->trylock()) return;
  auto& d=status_->msg_.data;d[0]=now;d[1]=static_cast<int>(phase);d[2]=static_cast<int>(core_.reason());d[3]=age;
  for(std::size_t j=0;j<7;++j) {
    d[4+j]=q[j];d[11+j]=dq[j];d[18+j]=core_.reference().q[j];d[25+j]=core_.reference().dq[j];
    d[32+j]=output_.raw[j];d[39+j]=output_.command[j];d[46+j]=output_.limited[j]?1.:0.;
  }
  d[53]=static_cast<int>(torque_fault_);d[54]=core_.sequence();d[55]=core_.pending_sequence();
  d[56]=core_.last_candidate();d[57]=static_cast<int>(core_.rejection());d[58]=core_.handoffs();
  d[59]=core_.remaining(now);d[60]=core_.observation_time();d[61]=core_.position_gap();d[62]=core_.velocity_gap();
  d[63]=static_cast<double>(invalid_packets_.load());d[64]=robot;d[65]=session_seconds_;
  // Append-only diagnostics: exact SDK period, subtraction of converted
  // seconds, host period, cumulative robot/host clock difference.
  for(std::size_t i=0;i<4;++i)d[66+i]=core_.clock_sample()[i];
  status_->unlockAndPublish();last_status_=now;last_status_phase_=phase;
  last_status_sequence_=core_.sequence();last_status_candidate_=core_.last_candidate();
}
controller_interface::return_type ContinuousEffortController::fault(TorqueFault reason,double now,double age,
                                                                   double robot,const Joints& q,const Joints& dq) {
  if(torque_fault_==TorqueFault::None) torque_fault_=reason;
  request_deactivate_.store(true);publish_status(now,age,robot,q,dq);return controller_interface::return_type::ERROR;
}
controller_interface::return_type ContinuousEffortController::update(const rclcpp::Time&,const rclcpp::Duration&) {
  const auto steady=steady_clock_ns();const double now=steady*1e-9;
  const auto receipt=heartbeat_receipt_ns_.load();const double age=receipt>0?std::max(0.,(steady-receipt)*1e-9):-1.;
  Joints q{},dq{};
  for(std::size_t j=0;j<7;++j) {q[j]=state_interfaces_[q_index_[j]].get_value();dq[j]=state_interfaces_[dq_index_[j]].get_value();}
  auto* robot=continuous_pointer<franka::RobotState>(state_interfaces_[state_index_].get_value());
  const double rt=robot?robot->time.toSec():0.;
  if(torque_fault_!=TorqueFault::None) return fault(torque_fault_,now,age,rt,q,dq);
  if(!robot) return fault(TorqueFault::Model,now,age,rt,q,dq);
  const bool first=first_update_;
  if(first) {
    if(!core_.arm_milliseconds(q,dq,expected_,robot->time.toMSec(),now,session_seconds_,task_rollout_)) return fault(TorqueFault::ReferenceOrState,now,age,rt,q,dq);
    first_update_=false;
  } else if(!core_.step_milliseconds(robot->time.toMSec(),now,age,q,dq,finish_.load())) return fault(TorqueFault::ReferenceOrState,now,age,rt,q,dq);
  const auto* candidate=candidates_.readFromRT();
  if(candidate->available&&candidate->raw.sequence>last_read_sequence_) {
    last_read_sequence_=candidate->raw.sequence;core_.admit(*candidate,now);
  }
  const double period=first?.001:core_.robot_period_seconds();
  Joints coriolis{};
  try {coriolis=model_->getCoriolisForceVector();}
  catch(const std::exception&) {return fault(TorqueFault::Model,now,age,rt,q,dq);}
  if(!law_.evaluate(core_.reference(),q,dq,coriolis,robot->tau_J_d,period,output_))
    return fault(law_.fault(),now,age,rt,q,dq);
  for(std::size_t j=0;j<7;++j) command_interfaces_[command_index_[j]].set_value(output_.command[j]);
  if(core_.phase()==Phase::Complete||core_.phase()==Phase::LatchedHold) request_deactivate_.store(true);
  publish_status(now,age,rt,q,dq);return controller_interface::return_type::OK;
}
}  // namespace flash_fr3_model_controller
PLUGINLIB_EXPORT_CLASS(flash_fr3_model_controller::ContinuousEffortController,controller_interface::ControllerInterface)
