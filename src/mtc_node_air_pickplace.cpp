#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("openarm_mtc_node");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("openarm_mtc_node", options) }
{
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error initializing stages:\n" << e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("openarm pick-place motion only (no objects)");
  task.loadRobotModel(node_);

  // ----- confirmed against your openarm_bimanual.srdf -----
  const auto& arm_group_name = "right_arm";
  const auto& hand_group_name = "right_gripper";
  const auto& hand_frame = "openarm_right_hand";
  const auto& root_frame = "openarm_body_link0";   // confirmed real root link -- no "world" frame exists

  task.setProperty("group", arm_group_name);
  // No "eef"/"ik_frame" task properties -- this task never calls
  // GenerateGraspPose/ComputeIK (no real object to generate grasps around),
  // so none of that machinery is needed. Every "grasp"/"place" step here is
  // just a plain relative move plus a named gripper state.

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(stage_state_current));

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.3);
  cartesian_planner->setMaxAccelerationScalingFactor(0.3);
  cartesian_planner->setStepSize(.01);

  // ----- move away from the singular "home" pose first -----
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move away from singularity", interpolation_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("hands_up");
    task.add(std::move(stage));
  }

  // ----- open gripper -----
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  // ----- move to "pick spot" (just a relative move, no object there) -----
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("move to pick spot", cartesian_planner);
    stage->setGroup(arm_group_name);
    stage->setIKFrame(hand_frame);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.05, 0.15);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = root_frame;
    vec.vector.x = 0.15;   // <-- adjust: wherever "in front" actually is for this arm's mounting
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  // ----- "close" gripper (mimics grasping, nothing actually there) -----
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("closed");
    task.add(std::move(stage));
  }

  // ----- lift -----
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian_planner);
    stage->setGroup(arm_group_name);
    stage->setIKFrame(hand_frame);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.05, 0.12);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = root_frame;
    vec.vector.z = 0.10;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  // ----- move to "place spot" -----
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("move to place spot", cartesian_planner);
    stage->setGroup(arm_group_name);
    stage->setIKFrame(hand_frame);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.05, 0.15);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = root_frame;
    vec.vector.y = 0.15;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  // ----- "open" gripper (mimics releasing) -----
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand (release)", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  // ----- retreat -----
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    stage->setGroup(arm_group_name);
    stage->setIKFrame(hand_frame);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.05, 0.15);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = root_frame;
    vec.vector.x = -0.15;
    vec.vector.y = -0.15;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

  // ----- return home -----
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
