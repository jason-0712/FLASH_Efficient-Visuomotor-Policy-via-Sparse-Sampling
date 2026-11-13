// Isolated ROS/controller interfaces with prescribed states and model output.
// No robot driver, network connection to FR3, or dynamics simulation.
#include <gtest/gtest.h>
#include <chrono>
#include <cstring>
#include <thread>
#include <franka/robot_state.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include "flash_fr3_model_controller/torque_controller.hpp"
using namespace flash_fr3_model_controller;

class PrescribedModel : public franka_hardware::Model {
 public:
  PrescribedModel() : franka_hardware::Model(nullptr) {}
  mutable int calls{0};bool invalid{false};
  std::array<double,7> coriolis(const franka::RobotState&) const override {
    ++calls;Joints out{};out.fill(.01);if(invalid)out[6]=NAN;return out;
  }
};
template<class T> double encoded_pointer(T* pointer) {
  double out;static_assert(sizeof(out)==sizeof(pointer));std::memcpy(&out,&pointer,sizeof(out));return out;
}
void native_fixture(const std::string& scenario) {
  rclcpp::init(0,nullptr);
  Joints q{-.16,-.77,.11,-2.38,.07,1.57,.66},dq{},commands{};
  commands.fill(.123);const auto untouched=commands;
  auto expected=std::vector<double>(q.begin(),q.end());
  if(scenario=="changed_start")expected[0]+=.003;
  const bool probe=scenario=="velocity_probe";
  std::vector<double> coefficients(98,0.);
  if(probe) {
    coefficients[3*7+3]=.010;coefficients[4*7+3]=-.015;coefficients[5*7+3]=.006;
    coefficients[49+3]=.001;
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("expected_start_q",expected),
    rclcpp::Parameter("segment_durations",std::vector<double>{probe?3.:2.,.75}),
    rclcpp::Parameter("segment_coefficients",coefficients),
    rclcpp::Parameter("k_gains",std::vector<double>(7,100.)),
    rclcpp::Parameter("d_gains",std::vector<double>(7,10.)),
    rclcpp::Parameter("torque_limits_nm",std::vector<double>(7,4.)),
    rclcpp::Parameter("torque_rates_nm_s",std::vector<double>(7,100.)),
    rclcpp::Parameter("allow_activation",scenario!="disabled"),
    rclcpp::Parameter("plan_id","isolated-torque-test-plan")});
  auto controller=std::make_shared<TorqueController>();
  ASSERT_EQ(controller->init("flash_fr3_torque","/",options),controller_interface::return_type::OK);
  ASSERT_EQ(controller->configure().id(),2);
  const auto required=controller->command_interface_configuration().names;
  ASSERT_EQ(required.size(),7u);for(const auto& name:required)EXPECT_NE(name.find("/effort"),std::string::npos);
  franka::RobotState robot;robot.q=q;robot.q_d.fill(0.);  // q_d deliberately stale.
  robot.m_ee=.73;robot.m_load=.27;robot.m_total=1.;robot.control_command_success_rate=.999;
  for(std::size_t j=0;j<7;++j) {
    robot.tau_J[j]=2.+j;robot.tau_ext_hat_filtered[j]=-.2*(j+1);
  }
  PrescribedModel model;model.invalid=scenario=="invalid_model";
  auto state_pointer=encoded_pointer(&robot),model_pointer=encoded_pointer(&model);
  if(scenario=="null_model")model_pointer=0.;
  std::vector<hardware_interface::StateInterface> state_handles;
  std::vector<hardware_interface::CommandInterface> command_handles;
  state_handles.reserve(16);command_handles.reserve(7);
  for(int j=6;j>=0;--j) {
    const auto joint="fr3_joint"+std::to_string(j+1);
    state_handles.emplace_back(joint,"velocity",&dq[j]);state_handles.emplace_back(joint,"position",&q[j]);
    command_handles.emplace_back(joint,"effort",&commands[j]);
  }
  state_handles.emplace_back("fr3","robot_model",&model_pointer);
  state_handles.emplace_back("fr3","robot_state",&state_pointer);
  std::vector<hardware_interface::LoanedStateInterface> states;
  std::vector<hardware_interface::LoanedCommandInterface> outputs;
  for(auto& h:state_handles)states.emplace_back(h);
  for(auto& h:command_handles)outputs.emplace_back(h);
  controller->assign_interfaces(std::move(outputs),std::move(states));
  const auto activation=controller->on_activate(controller->get_state());
  if(scenario=="disabled"||scenario=="null_model") {
    EXPECT_EQ(activation,controller_interface::CallbackReturn::FAILURE);EXPECT_EQ(commands,untouched);
    controller->release_interfaces();controller.reset();rclcpp::shutdown();return;
  }
  ASSERT_EQ(activation,controller_interface::CallbackReturn::SUCCESS);
  auto peer=std::make_shared<rclcpp::Node>("torque_service_fixture");int stop_requests=0;
  std::vector<std::vector<double>> sdk_messages;
  auto sdk_subscription=peer->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/flash_fr3_torque/sdk_status",rclcpp::QoS(10),
    [&](std_msgs::msg::Float64MultiArray::ConstSharedPtr message) {sdk_messages.push_back(message->data);});
  auto service=peer->create_service<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller",[&](
        controller_manager_msgs::srv::SwitchController::Request::SharedPtr request,
        controller_manager_msgs::srv::SwitchController::Response::SharedPtr response) {
      ++stop_requests;EXPECT_TRUE(request->activate_controllers.empty());
      EXPECT_EQ(request->deactivate_controllers,std::vector<std::string>{"flash_fr3_torque"});response->ok=true;
    });
  auto beat=peer->create_publisher<std_msgs::msg::Header>("/flash_fr3_torque/heartbeat",rclcpp::QoS(1).best_effort());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller->get_node()->get_node_base_interface());executor.add_node(peer);
  const auto discovery=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(beat->get_subscription_count()==0&&std::chrono::steady_clock::now()<discovery) {
    executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_GT(beat->get_subscription_count(),0u);
  std_msgs::msg::Header heartbeat;heartbeat.stamp=rclcpp::Clock(RCL_SYSTEM_TIME).now();
  heartbeat.frame_id="isolated-torque-test-plan";beat->publish(heartbeat);
  for(int i=0;i<5;++i){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  auto update=controller->update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.001));
  if(scenario=="changed_start"||scenario=="invalid_model") {
    EXPECT_EQ(update,controller_interface::return_type::ERROR);EXPECT_EQ(commands,untouched);
    // Even a repaired input cannot restart a latched failed controller.
    model.invalid=false;robot.time=franka::Duration(1);
    EXPECT_EQ(controller->update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.001)),controller_interface::return_type::ERROR);
    EXPECT_EQ(commands,untouched);
  } else {
    ASSERT_EQ(update,controller_interface::return_type::OK);
    for(double command:commands)EXPECT_NEAR(command,.01,1e-12);
    for(int i=1;i<=(probe?3760:2760);++i) {
      robot.time=franka::Duration(static_cast<uint64_t>(i));robot.tau_J_d=commands;
      if(scenario=="period_fault"&&i==1)robot.time=franka::Duration(3);
      update=controller->update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.001));
      if(scenario=="period_fault") {EXPECT_EQ(update,controller_interface::return_type::ERROR);break;}
      ASSERT_EQ(update,controller_interface::return_type::OK);
      // Prescribed measured q stays fixed: no dynamics simulation. At the
      // quintic midpoint, q_ref-q=.0005 and analytic dq_ref=.000625.
      if(probe&&i==1500) {
        EXPECT_NEAR(commands[3],.01+100.*.0005+10.*.000625,1e-10);
      }
    }
    EXPECT_GT(model.calls,0);
    for(std::size_t j=0;j<7;++j) EXPECT_NEAR(commands[j],probe&&j==3?.11:.01,1e-10);
  }
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(stop_requests==0&&std::chrono::steady_clock::now()<deadline) {
    executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(stop_requests,1);
  if(scenario=="complete"||probe) {
    const auto telemetry_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
    while((sdk_messages.empty()||sdk_messages.back()[3]!=4.)&&std::chrono::steady_clock::now()<telemetry_deadline) {
      executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_FALSE(sdk_messages.empty());const auto& data=sdk_messages.back();
    ASSERT_EQ(data.size(),66u);EXPECT_EQ(data[0],1.);EXPECT_EQ(data[3],4.);EXPECT_EQ(data[4],1.);
    EXPECT_NEAR(data[5],.001,1e-12);
    for(std::size_t j=0;j<7;++j) {
      // Permuted loaned interfaces must not permute SDK joint ordering. The
      // observer reports the prior SDK command separately from the new write.
      EXPECT_DOUBLE_EQ(data[6+j],robot.tau_J_d[j]);
      EXPECT_DOUBLE_EQ(data[13+j],robot.tau_J_d[j]);
      EXPECT_DOUBLE_EQ(data[20+j],commands[j]);
      EXPECT_DOUBLE_EQ(data[27+j],robot.tau_J[j]);
      EXPECT_DOUBLE_EQ(data[34+j],robot.tau_ext_hat_filtered[j]);
      EXPECT_DOUBLE_EQ(data[41+j],.01);EXPECT_DOUBLE_EQ(data[48+j],q[j]);EXPECT_DOUBLE_EQ(data[55+j],0.);
    }
    EXPECT_DOUBLE_EQ(data[62],.73);EXPECT_DOUBLE_EQ(data[63],.27);EXPECT_DOUBLE_EQ(data[64],1.);
    EXPECT_DOUBLE_EQ(data[65],.999);
  }
  EXPECT_EQ(controller->on_deactivate(controller->get_state()),controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(controller->on_activate(controller->get_state()),controller_interface::CallbackReturn::FAILURE);
  executor.remove_node(controller->get_node()->get_node_base_interface());executor.remove_node(peer);
  controller->release_interfaces();controller.reset();rclcpp::shutdown();
}
TEST(TorqueNative, EffortInterfacesMeasuredOriginCoriolisAndNativeCompletion) {native_fixture("complete");}
TEST(TorqueNative, ActivationDisabledByDefaultAdmission) {native_fixture("disabled");}
TEST(TorqueNative, NullModelCannotActivate) {native_fixture("null_model");}
TEST(TorqueNative, ChangedStartNoWriteAndNativeStop) {native_fixture("changed_start");}
TEST(TorqueNative, InvalidModelNoPartialWriteAndNativeStop) {native_fixture("invalid_model");}
TEST(TorqueNative, PeriodFaultLatchesAndRequestsNativeStop) {native_fixture("period_fault");}
TEST(TorqueNative, NonzeroAnalyticVelocityReachesEffortAndStopsNatively) {native_fixture("velocity_probe");}
