// Isolated ROS interfaces, prescribed states/model/clock; no FR3 driver,
// robot connection, plant dynamics, or physical timing performance claim.
#include <gtest/gtest.h>
#include <chrono>
#include <cstring>
#include <thread>
#include <franka/robot_state.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include "flash_fr3_model_controller/continuous_effort_controller.hpp"
using namespace flash_fr3_model_controller;
namespace {
int64_t actual_steady() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
class PrescribedClockController : public ContinuousEffortController {
 public:
  int64_t now=actual_steady();
 protected:
  int64_t steady_clock_ns()const override{return now;}
};
class ContinuousPrescribedModel : public franka_hardware::Model {
 public:
  ContinuousPrescribedModel():franka_hardware::Model(nullptr){}
  bool invalid{false};mutable int calls{};
  std::array<double,7> coriolis(const franka::RobotState&)const override {
    ++calls;Joints out{};out.fill(.01);if(invalid)out[6]=NAN;return out;
  }
};
template<class T> double encode_pointer(T* p) {
  double out;static_assert(sizeof(out)==sizeof(p));std::memcpy(&out,&p,sizeof(out));return out;
}
void native_case(const std::string& scenario) {
  rclcpp::init(0,nullptr);
  const std::string id="isolated-continuous-effort-session";
  Joints q{-.16,-.77,.11,-2.38,.07,1.57,.66},dq{},commands{};commands.fill(.123);
  const auto untouched=commands;
  const auto original_q=q;
  const bool rolling=scenario=="twenty"||scenario=="rollout";
  auto expected=std::vector<double>(q.begin(),q.end());if(scenario=="changed_start")expected[1]+=.003;
  rclcpp::NodeOptions options;
  std::vector<rclcpp::Parameter> parameters{
    rclcpp::Parameter("expected_start_q",expected),rclcpp::Parameter("session_id",id),
    rclcpp::Parameter("maximum_session_seconds",8.),
    rclcpp::Parameter("execution_profile",(scenario=="rollout"||scenario=="startup_anchor"||scenario=="late_recovery")?"raw_policy_rollout":"commissioning"),
    rclcpp::Parameter("k_gains",std::vector<double>(7,100.)),
    rclcpp::Parameter("d_gains",std::vector<double>(7,10.)),
    rclcpp::Parameter("torque_limits_nm",std::vector<double>(7,4.)),
    rclcpp::Parameter("torque_rates_nm_s",std::vector<double>(7,100.))};
  if(scenario!="disabled")parameters.emplace_back("allow_activation",true);
  options.parameter_overrides(parameters);
  auto controller=std::make_shared<PrescribedClockController>();
  ASSERT_EQ(controller->init("flash_fr3_continuous_effort","/",options),controller_interface::return_type::OK);
  ASSERT_EQ(controller->configure().id(),2);
  const auto required=controller->command_interface_configuration().names;
  ASSERT_EQ(required.size(),7u);for(const auto& name:required)EXPECT_NE(name.find("/effort"),std::string::npos);
  franka::RobotState robot;robot.q=q;robot.q_d.fill(0.);ContinuousPrescribedModel model;
  const uint64_t robot_base_ms=scenario=="sdk_period_boundary"?10491237:0;
  robot.time=franka::Duration(robot_base_ms);
  model.invalid=scenario=="invalid_model";
  auto state_pointer=encode_pointer(&robot),model_pointer=encode_pointer(&model);
  if(scenario=="null_model")model_pointer=0.;
  std::vector<hardware_interface::StateInterface> state_handles;
  std::vector<hardware_interface::CommandInterface> command_handles;
  state_handles.reserve(16);command_handles.reserve(7);
  for(int j=6;j>=0;--j) {
    const auto joint="fr3_joint"+std::to_string(j+1);
    state_handles.emplace_back(joint,"velocity",&dq[j]);state_handles.emplace_back(joint,"position",&q[j]);
    command_handles.emplace_back(joint,"effort",&commands[j]);
  }
  state_handles.emplace_back("fr3","robot_model",&model_pointer);state_handles.emplace_back("fr3","robot_state",&state_pointer);
  std::vector<hardware_interface::LoanedStateInterface> states;
  std::vector<hardware_interface::LoanedCommandInterface> outputs;
  for(auto& h:state_handles)states.emplace_back(h);for(auto& h:command_handles)outputs.emplace_back(h);
  controller->assign_interfaces(std::move(outputs),std::move(states));
  const auto activated=controller->on_activate(controller->get_state());
  if(scenario=="disabled"||scenario=="null_model") {
    EXPECT_EQ(activated,controller_interface::CallbackReturn::FAILURE);EXPECT_EQ(commands,untouched);
    controller->release_interfaces();controller.reset();rclcpp::shutdown();return;
  }
  ASSERT_EQ(activated,controller_interface::CallbackReturn::SUCCESS);
  const int64_t base=controller->now;const double start=base*1e-9;
  auto peer=std::make_shared<rclcpp::Node>("continuous_effort_fixture");int native_stops=0;
  std::vector<double> last_status;int observed_handoffs=0;double largest_dq_ref=0.;
  auto state_subscription=peer->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/flash_fr3_continuous_effort/status",rclcpp::QoS(100),[&](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
      ASSERT_EQ(msg->data.size(),70u);last_status=msg->data;
      observed_handoffs=std::max(observed_handoffs,static_cast<int>(last_status[58]));
      largest_dq_ref=std::max(largest_dq_ref,std::abs(last_status[28]));
    });
  auto stop_service=peer->create_service<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller",[&](
      controller_manager_msgs::srv::SwitchController::Request::SharedPtr request,
      controller_manager_msgs::srv::SwitchController::Response::SharedPtr response) {
        ++native_stops;EXPECT_TRUE(request->activate_controllers.empty());
        EXPECT_EQ(request->deactivate_controllers,std::vector<std::string>{"flash_fr3_continuous_effort"});response->ok=true;
      });
  auto heartbeat=peer->create_publisher<std_msgs::msg::Header>("/flash_fr3_continuous_effort/heartbeat",rclcpp::QoS(1).best_effort());
  auto candidate=peer->create_publisher<std_msgs::msg::Float64MultiArray>("/flash_fr3_continuous_effort/candidate",rclcpp::QoS(1).reliable());
  auto finish=peer->create_publisher<std_msgs::msg::Header>("/flash_fr3_continuous_effort/finish",rclcpp::QoS(1).reliable());
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(peer);
  executor.add_node(controller->get_node()->get_node_base_interface());
  auto spin=[&](int count=3) {
    for(int i=0;i<count;++i) {executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  };
  const auto discovery=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while((candidate->get_subscription_count()==0||heartbeat->get_subscription_count()==0||finish->get_subscription_count()==0)&&
        std::chrono::steady_clock::now()<discovery)spin();
  ASSERT_GT(candidate->get_subscription_count(),0u);ASSERT_GT(heartbeat->get_subscription_count(),0u);
  auto beat=[&] {
    std_msgs::msg::Header h;h.frame_id=id;h.stamp=rclcpp::Clock(RCL_SYSTEM_TIME).now();heartbeat->publish(h);spin(2);
  };
  auto send=[&](int sequence) {
    const double observation=start+sequence*4./15.,velocity=scenario=="startup_anchor"?.02:scenario=="rollout"?.01:scenario=="twenty"?.00005:0.;
    PhysicalCurve curve;curve.legendre[0]=original_q;curve.legendre[1][3]=velocity*curve.duration/2.;
    curve.legendre[0][3]+=velocity*(observation-start)-curve.legendre[1][3]*(2*curve.anchor-1.);
    if(scenario=="jump")curve.legendre[0][2]+=.001;
    std_msgs::msg::Float64MultiArray msg;msg.layout.dim.resize(1);msg.layout.dim[0].size=57;msg.layout.dim[0].stride=57;
    msg.layout.dim[0].label=scenario=="wrong_session"?"different-session-identifier":id;
    msg.data={1.,static_cast<double>(sequence),rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds(),
              observation,observation+.1,curve.anchor,curve.duration,curve.horizon};
    for(const auto& row:curve.legendre)for(double value:row)msg.data.push_back(value);
    if(scenario=="late_recovery"&&sequence==1)msg.data[4]=controller->now*1e-9+.005;
    if(scenario=="startup_anchor") {
      msg.data[0]=2.;msg.data[3]=start-.4;msg.data.push_back(start+.1);
      msg.layout.dim[0].size=58;msg.layout.dim[0].stride=58;
    }
    candidate->publish(msg);
    ASSERT_TRUE(candidate->wait_for_all_acked(std::chrono::seconds(1)));
    spin(4);
  };
  beat();
  auto result=controller->update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.001));
  if(scenario=="invalid_model"||scenario=="changed_start") {
    EXPECT_EQ(result,controller_interface::return_type::ERROR);EXPECT_EQ(commands,untouched);
    model.invalid=false;controller->now+=1000000;robot.time=franka::Duration(1);
    EXPECT_EQ(controller->update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.001)),controller_interface::return_type::ERROR);
    EXPECT_EQ(commands,untouched);
  } else {
    ASSERT_EQ(result,controller_interface::return_type::OK);for(double value:commands)EXPECT_NEAR(value,.01,1e-12);
    send(0);int next=1;
    const int end=rolling?6505:scenario=="heartbeat"?1200:2300;
    for(int ms=1;ms<=end;++ms) {
      const int elapsed_ms=ms+(scenario=="sdk_period_boundary"?1:0);
      controller->now=base+static_cast<int64_t>(elapsed_ms)*1000000;
      robot.time=franka::Duration(robot_base_ms+static_cast<uint64_t>(elapsed_ms));robot.tau_J_d=commands;
      if(scenario=="rollout"&&ms>=100) {
        q[3]=original_q[3]+.01*ms*.001;dq[3]=.01;robot.q=q;robot.dq=dq;
      }
      if(scenario=="period_fault"&&ms==1)robot.time=franka::Duration(3);
      if(ms%20==0&&(scenario!="heartbeat"||ms<300))beat();
      // Availability must be strictly after the source timestamp. Exact grid
      // equality can differ by one floating-point ULP after ns conversion.
      if(rolling&&next<=20&&ms>=std::ceil(next*4./15.*1000)+1)send(next++);
      if(scenario=="late_recovery"&&(ms==300||ms==540))send(ms==300?1:2);
      // Let DDS and the realtime publisher run even after the heartbeat cut;
      // these are prescribed clock ticks, not a wall-time speed benchmark.
      if(ms%20==0)spin(1);
      if((rolling&&ms==6000)||(scenario=="late_recovery"&&ms==1600)) {
        std_msgs::msg::Header h;h.frame_id=id;h.stamp=rclcpp::Clock(RCL_SYSTEM_TIME).now();finish->publish(h);spin(4);
      }
      result=controller->update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.001));
      if(scenario=="period_fault") {EXPECT_EQ(result,controller_interface::return_type::ERROR);break;}
      ASSERT_EQ(result,controller_interface::return_type::OK);
      if(scenario=="late_recovery"&&ms==310) {
        spin(4);ASSERT_EQ(last_status.size(),70u);
        EXPECT_EQ(static_cast<int>(last_status[56]),1);
        EXPECT_EQ(static_cast<int>(last_status[57]),3);
        EXPECT_EQ(static_cast<int>(last_status[63]),0);
      }
      if(scenario=="sdk_period_boundary"&&ms==1) {
        spin(4);ASSERT_EQ(last_status.size(),70u);
        EXPECT_DOUBLE_EQ(last_status[66],.002);EXPECT_GT(last_status[67],.002);
      }
      if(scenario=="startup_anchor"&&ms==150) {
        spin(4);ASSERT_EQ(last_status.size(),70u);
        EXPECT_EQ(static_cast<int>(last_status[54]),0);
        EXPECT_NEAR(last_status[60],start-.4,1e-8);
        EXPECT_NEAR(commands[3],.01+100.*(.02*.05)+10.*.02,1e-9);
      }
      if(scenario=="twenty"&&ms==1500)EXPECT_NEAR(commands[3],.01+100.*(.00005*1.5)+10.*.00005,1e-10);
    }
    EXPECT_GT(model.calls,0);
  }
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(native_stops==0&&std::chrono::steady_clock::now()<deadline)spin();
  spin(5);EXPECT_EQ(native_stops,1);ASSERT_EQ(last_status.size(),70u);
  if(scenario=="period_fault") {
    EXPECT_EQ(static_cast<int>(last_status[2]),3);EXPECT_DOUBLE_EQ(last_status[66],.003);
  }
  if(scenario=="sdk_period_boundary") {
    EXPECT_EQ(static_cast<int>(last_status[53]),0);EXPECT_EQ(static_cast<int>(last_status[2]),8);
  }
  if(rolling) {
    EXPECT_EQ(observed_handoffs,20)<<"last sequence="<<last_status[54]<<", candidate="<<last_status[56]
      <<", rejection="<<last_status[57]<<", invalid="<<last_status[63];
    EXPECT_GT(largest_dq_ref,0.);
    EXPECT_EQ(static_cast<int>(last_status[1]),4);EXPECT_EQ(static_cast<int>(last_status[2]),10);
    EXPECT_NEAR(last_status[28],0.,1e-12);
  } else if(scenario=="late_recovery") {
    EXPECT_EQ(static_cast<int>(last_status[1]),4);EXPECT_EQ(static_cast<int>(last_status[2]),10);
    EXPECT_EQ(static_cast<int>(last_status[54]),2);EXPECT_EQ(observed_handoffs,1);
  } else if(scenario=="heartbeat") {
    EXPECT_EQ(static_cast<int>(last_status[1]),3);EXPECT_EQ(static_cast<int>(last_status[2]),1);
  } else if(scenario=="expiry") {
    EXPECT_EQ(static_cast<int>(last_status[1]),3);EXPECT_EQ(static_cast<int>(last_status[2]),8);
  } else if(scenario=="wrong_session"||scenario=="jump") {
    EXPECT_EQ(static_cast<int>(last_status[54]),-1);EXPECT_EQ(static_cast<int>(last_status[2]),2);
    if(scenario=="wrong_session")EXPECT_GE(last_status[63],1.);
    else EXPECT_EQ(static_cast<int>(last_status[57]),6);
    for(int j=0;j<7;++j)EXPECT_NEAR(last_status[18+j],q[j],1e-12);
  }
  EXPECT_EQ(controller->on_deactivate(controller->get_state()),controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(controller->on_activate(controller->get_state()),controller_interface::CallbackReturn::FAILURE);
  executor.remove_node(controller->get_node()->get_node_base_interface());executor.remove_node(peer);
  controller->release_interfaces();controller.reset();rclcpp::shutdown();
}
}
TEST(ContinuousEffortNative,TwentyRawUpdatesAnalyticVelocityEffortAndNativeFinish){native_case("twenty");}
TEST(ContinuousEffortNative,HeartbeatCutStopsWithoutClientSwitchRequest){native_case("heartbeat");}
TEST(ContinuousEffortNative,MissingPredictionBrakesAndStopsNatively){native_case("expiry");}
TEST(ContinuousEffortNative,WrongSessionPacketCannotStartMotion){native_case("wrong_session");}
TEST(ContinuousEffortNative,RawBoundaryJumpIsRejectedWithoutRepair){native_case("jump");}
TEST(ContinuousEffortNative,InvalidModelPreventsEveryCommandWrite){native_case("invalid_model");}
TEST(ContinuousEffortNative,RobotPeriodFaultIsLatched){native_case("period_fault");}
TEST(ContinuousEffortNative,ChangedStartPreventsEveryCommandWrite){native_case("changed_start");}
TEST(ContinuousEffortNative,ActivationDisabledByDefault){native_case("disabled");}
TEST(ContinuousEffortNative,NullModelPreventsActivation){native_case("null_model");}
TEST(ContinuousEffortNative,RawPolicyProfileRunsTwentyUpdatesAboveMicroEnvelope){native_case("rollout");}
TEST(ContinuousEffortNative,VersionTwoStartupKeepsSourceTimeAndStartsAtAnchor){native_case("startup_anchor");}
TEST(ContinuousEffortNative,ExactTwoMillisecondSdkPeriodAtLongUptimeDoesNotFault){native_case("sdk_period_boundary");}
TEST(ContinuousEffortNative,LateCandidateReturnsClockRejectionAndNextUpdateCanRun){native_case("late_recovery");}
