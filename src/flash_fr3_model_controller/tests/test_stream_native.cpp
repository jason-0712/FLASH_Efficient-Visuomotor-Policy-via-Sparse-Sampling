// ROS transport + native stop integration only; no driver or robot dynamics.
#include <gtest/gtest.h>
#include <chrono>
#include <cstring>
#include <thread>
#include <franka/robot_state.h>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include "flash_fr3_model_controller/stream_controller.hpp"
using namespace flash_fr3_model_controller;

void fixture(bool late,int replacements=1,bool late_second=false) {
  rclcpp::init(0,nullptr);
  Joints q{-.16,-.78,.11,-2.38,.07,1.56,.66},dq{},command{};
  Plan plan;plan[0].duration=.5;plan[0].c[3][1]=.0002;
  plan[1]=stopping_polynomial(plan[0].at(.5),.5);
  std::vector<double> coefficients;
  for (auto& p:plan) for (auto& row:p.c) for(double x:row) coefficients.push_back(x);
  rclcpp::NodeOptions options;options.parameter_overrides({
    rclcpp::Parameter("expected_start_q",std::vector<double>(q.begin(),q.end())),
    rclcpp::Parameter("segment_durations",std::vector<double>{.5,.5}),
    rclcpp::Parameter("segment_coefficients",coefficients),
    rclcpp::Parameter("plan_id","isolated-stream-session"),
    rclcpp::Parameter("max_replacements",replacements)});
  auto c=std::make_shared<StreamController>();
  ASSERT_EQ(c->init("flash_fr3_stream","/",options),controller_interface::return_type::OK);
  ASSERT_EQ(c->configure().id(),2);
  franka::RobotState robot;robot.q_d=q;auto* ptr=&robot;double encoded;
  std::memcpy(&encoded,&ptr,sizeof(encoded));
  std::vector<hardware_interface::StateInterface> sh;
  std::vector<hardware_interface::CommandInterface> ch;sh.reserve(15);ch.reserve(7);
  for (int j=6;j>=0;--j) {
    const auto name="fr3_joint"+std::to_string(j+1);
    sh.emplace_back(name,"velocity",&dq[j]);sh.emplace_back(name,"position",&q[j]);
    ch.emplace_back(name,"position",&command[j]);
  }
  sh.emplace_back("fr3","robot_state",&encoded);
  std::vector<hardware_interface::LoanedStateInterface> states;
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  for (auto& s:sh) states.emplace_back(s);
  for (auto& s:ch) commands.emplace_back(s);
  c->assign_interfaces(std::move(commands),std::move(states));
  ASSERT_EQ(c->on_activate(c->get_state()),controller_interface::CallbackReturn::SUCCESS);
  auto n=std::make_shared<rclcpp::Node>("stream_transport_fixture");int stops=0,sequence=-1,rejection=-1;
  auto service=n->create_service<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller",[&](
      controller_manager_msgs::srv::SwitchController::Request::SharedPtr request,
      controller_manager_msgs::srv::SwitchController::Response::SharedPtr response){
        EXPECT_EQ(request->deactivate_controllers,std::vector<std::string>{"flash_fr3_stream"});
        EXPECT_TRUE(request->activate_controllers.empty());++stops;response->ok=true;});
  auto hb=n->create_publisher<std_msgs::msg::Header>("/flash_fr3_stream/heartbeat",rclcpp::QoS(1).best_effort());
  auto pub=n->create_publisher<std_msgs::msg::Float64MultiArray>("/flash_fr3_stream/candidate",rclcpp::QoS(1).reliable());
  auto sub=n->create_subscription<std_msgs::msg::Float64MultiArray>("/flash_fr3_stream/status",10,
      [&](std_msgs::msg::Float64MultiArray::ConstSharedPtr m){
        ASSERT_EQ(m->data.size(),53u);sequence=static_cast<int>(m->data[42]);rejection=static_cast<int>(m->data[45]);});
  rclcpp::executors::SingleThreadedExecutor ex;ex.add_node(c->get_node()->get_node_base_interface());ex.add_node(n);
  auto spin=[&](int ms){auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);
    do {ex.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));} while(std::chrono::steady_clock::now()<until);};
  for(int i=0;i<100 && (pub->get_subscription_count()==0||hb->get_subscription_count()==0);++i) spin(10);
  ASSERT_GT(pub->get_subscription_count(),0u);ASSERT_GT(hb->get_subscription_count(),0u);
  std_msgs::msg::Header beat;beat.stamp=rclcpp::Clock(RCL_SYSTEM_TIME).now();beat.frame_id="isolated-stream-session";
  hb->publish(beat);spin(5);
  const auto boundary=plan[0].at(.5);Plan next;
  next[0].duration=.5;next[0].c[0]=boundary.q;
  for(int j=0;j<7;++j){next[0].c[1][j]=.5*boundary.dq[j];next[0].c[2][j]=.125*boundary.ddq[j];}
  next[0].c[3][1]=-.0001;next[1]=stopping_polynomial(next[0].at(.5),.5);
  const auto second_boundary=next[0].at(.5);Plan last;
  last[0].duration=.5;last[0].c[0]=second_boundary.q;
  for(int j=0;j<7;++j){last[0].c[1][j]=.5*second_boundary.dq[j];last[0].c[2][j]=.125*second_boundary.ddq[j];}
  last[0].c[3][1]=-.0001;last[1]=stopping_polynomial(last[0].at(.5),.5);
  for(int i=0;i<=2010;++i) {
    const bool send_second=replacements==2 && i==(late_second?1050:600);
    if(i==(late?550:100) || send_second) {
      std_msgs::msg::Float64MultiArray m;m.layout.dim.resize(1);
      m.layout.dim[0].label="isolated-stream-session";m.layout.dim[0].size=m.layout.dim[0].stride=104;
      m.data={1.,send_second?2.:1.,send_second?1.:.5,
              rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds(),.5,.5};
      const auto& candidate=send_second?last:next;
      for(auto& p:candidate) for(auto& row:p.c) for(double v:row)m.data.push_back(v);
      pub->publish(m);spin(5);
    }
    robot.time=franka::Duration(static_cast<uint64_t>(i));
    ASSERT_EQ(c->update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.001)),controller_interface::return_type::OK);
  }
  spin(100);
  EXPECT_EQ(stops,1);EXPECT_EQ(sequence,late?0:(replacements==2&&!late_second?2:1));
  EXPECT_EQ(rejection,late||late_second?static_cast<int>(Rejection::LateOrWrongTime):0);
  const double end=late?plan[1].at(.5).q[1]:
      (replacements==2&&!late_second?last[1].at(.5).q[1]:next[1].at(.5).q[1]);
  EXPECT_NEAR(command[1],q[1]+end,1e-12);
  c->on_deactivate(c->get_state());
  EXPECT_EQ(c->on_activate(c->get_state()),controller_interface::CallbackReturn::FAILURE);
  c->release_interfaces();ex.remove_node(c->get_node()->get_node_base_interface());ex.remove_node(n);
  c.reset();rclcpp::shutdown();
}
TEST(StreamNative, RealRosCandidateReplacesOnceAndCompletesWithNativeStop){fixture(false);}
TEST(StreamNative, LateRosCandidateCannotReplaceStop){fixture(true);}
TEST(StreamNative, TwoRosCandidatesCompleteWithOneNativeStop){fixture(false,2);}
TEST(StreamNative, LateSecondCandidateKeepsFirstReplacementStop){fixture(false,2,true);}
