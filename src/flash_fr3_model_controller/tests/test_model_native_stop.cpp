// Controller/service integration fixture, no hardware driver or dynamics.
#include <gtest/gtest.h>
#include <chrono>
#include <cstring>
#include <thread>
#include <franka/robot_state.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include "flash_fr3_model_controller/model_controller.hpp"

using namespace flash_fr3_model_controller;

void native_stop_fixture(bool stale_plan) {
  rclcpp::init(0, nullptr);
  Joints q{-.16,-.78,.11,-2.38,.07,1.56,.66}, dq{}, command{};
  std::vector<double> expected(q.begin(), q.end());
  if (stale_plan) expected[0] += .003;  // Deliberately stale plan triggers the native stop path.
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("expected_start_q", expected),
      rclcpp::Parameter("segment_durations", std::vector<double>{1., .75}),
      rclcpp::Parameter("segment_coefficients", std::vector<double>(98,0.)),
      rclcpp::Parameter("plan_id", "isolated-unit-test-plan")});
  auto c = std::make_shared<ModelController>();
  ASSERT_EQ(c->init("flash_fr3_model", "/", options), controller_interface::return_type::OK);
  ASSERT_EQ(c->configure().id(), 2);
  franka::RobotState robot;
  robot.q_d = q;
  auto* pointer = &robot;
  double encoded;
  std::memcpy(&encoded, &pointer, sizeof(encoded));
  std::vector<hardware_interface::StateInterface> state_handles;
  std::vector<hardware_interface::CommandInterface> command_handles;
  state_handles.reserve(15);
  command_handles.reserve(7);
  // Reverse ordering ensures all q/dq and command indices are name-based.
  for (int j = 6; j >= 0; --j) {
    const std::string joint = "fr3_joint" + std::to_string(j+1);
    state_handles.emplace_back(joint, "velocity", &dq[j]);
    state_handles.emplace_back(joint, "position", &q[j]);
    command_handles.emplace_back(joint, "position", &command[j]);
  }
  state_handles.emplace_back("fr3", "robot_state", &encoded);
  std::vector<hardware_interface::LoanedStateInterface> states;
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  for (auto& s : state_handles) states.emplace_back(s);
  for (auto& s : command_handles) commands.emplace_back(s);
  c->assign_interfaces(std::move(commands), std::move(states));
  ASSERT_EQ(c->on_activate(c->get_state()), controller_interface::CallbackReturn::SUCCESS);
  auto server = std::make_shared<rclcpp::Node>("native_stop_service_fixture");
  std::size_t calls = 0;
  auto service = server->create_service<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller", [&calls](
      controller_manager_msgs::srv::SwitchController::Request::SharedPtr request,
      controller_manager_msgs::srv::SwitchController::Response::SharedPtr response) {
        ++calls;
        EXPECT_EQ(request->deactivate_controllers, std::vector<std::string>{"flash_fr3_model"});
        EXPECT_TRUE(request->activate_controllers.empty());
        response->ok = true;
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(c->get_node()->get_node_base_interface());
  executor.add_node(server);
  if (stale_plan) {
    ASSERT_EQ(c->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.001)),
              controller_interface::return_type::ERROR);
  } else {
    auto heartbeat = server->create_publisher<std_msgs::msg::Header>("/flash_fr3_model/heartbeat",rclcpp::QoS(1).best_effort());
    const auto connected = std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while (heartbeat->get_subscription_count()==0 && std::chrono::steady_clock::now()<connected) {
      executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_GT(heartbeat->get_subscription_count(),0u);
    std_msgs::msg::Header message;
    message.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    message.frame_id = "isolated-unit-test-plan";
    heartbeat->publish(message);
    // Transport discovery and callback delivery precede the first RT update.
    for (int i=0; i<4; ++i) {executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    ASSERT_EQ(c->update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.001)),
              controller_interface::return_type::OK);
    for (int i=1; i<=1760; ++i) {
      robot.time = franka::Duration(static_cast<uint64_t>(i));
      ASSERT_EQ(c->update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.001)),
                controller_interface::return_type::OK);
    }
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline && calls == 0) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(calls, 1u);
  for (std::size_t j=0; j<7; ++j) EXPECT_DOUBLE_EQ(command[j],stale_plan?0.:q[j]);  // Never wrote a joint command.
  EXPECT_EQ(c->on_deactivate(c->get_state()), controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(c->on_activate(c->get_state()), controller_interface::CallbackReturn::FAILURE);
  executor.remove_node(c->get_node()->get_node_base_interface());
  executor.remove_node(server);
  c->release_interfaces();
  c.reset();
  rclcpp::shutdown();
}

TEST(ModelNativeStop, HardFaultRequestsDeactivationEvenWithoutPythonClient) { native_stop_fixture(true); }
TEST(ModelNativeStop, CompletionRequestsDeactivationEvenWithoutPythonClient) { native_stop_fixture(false); }
