#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("openarm_mtc_handover_node");
namespace mtc = moveit::task_constructor;

// This is a genuine handover, not two separate objects like the earlier
// bimanual attempt: ONE "stick" object. Right picks it up from its
// original spot and places it near the centerline; left then picks up
// THAT SAME OBJECT from wherever right left it (GenerateGraspPose reads
// the object's live pose from the planning scene automatically -- no
// coordinates need to be hardcoded for left's pick) and places it
// somewhere else again.
//
// Structured so either arm's block can be commented out to isolate/test
// it alone, same as the bimanual pick-and-place node.

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();
  void setupPlanningScene();

private:
  mtc::Task createTask();

  std::unique_ptr<mtc::SerialContainer> buildPickContainer(
      const std::string& side, const std::string& arm_group, const std::string& gripper_group,
      const std::string& gripper_joint_name, const std::string& eef_name, const std::string& hand_frame,
      const Eigen::Vector3d& grasp_rotation_axis, double grasp_rotation_angle, double grasp_z_offset,
      const std::shared_ptr<mtc::solvers::CartesianPath>& cartesian_planner,
      const std::shared_ptr<mtc::solvers::JointInterpolationPlanner>& interpolation_planner,
      mtc::Stage* current_state_ptr, mtc::Stage** attach_stage_out);

  std::unique_ptr<mtc::SerialContainer> buildPlaceContainer(
      const std::string& side, const std::string& arm_group, const std::string& gripper_group,
      const std::string& eef_name, const std::string& hand_frame,
      double place_offset_x, double place_offset_y,
      const std::shared_ptr<mtc::solvers::CartesianPath>& cartesian_planner,
      const std::shared_ptr<mtc::solvers::JointInterpolationPlanner>& interpolation_planner,
      mtc::Stage* attach_stage);

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("openarm_mtc_handover_node", options) }
{
}

void MTCTaskNode::setupPlanningScene()
{
  // Same bar geometry/position as the working single-arm task -- update
  // these to match your current tuned single-arm values if they've moved
  // on since.
  moveit_msgs::msg::CollisionObject stick;
  stick.id = "stick";
  stick.header.frame_id = "openarm_body_link0";
  stick.primitives.resize(1);
  stick.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  stick.primitives[0].dimensions = { 0.8, 0.02 };   // [height, radius]

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.3;
  pose.position.y = -0.3;
  pose.position.z = 0.4;
  pose.orientation.w = 1.0;
  stick.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(stick);
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

  // Handover has two full grasp/place generator chains chained sequentially
  // (right's, then left's) -- same reasoning as the bimanual task: this
  // needs a bigger solution budget than a single-arm task's 5.
  if (!task_.plan(50))
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

std::unique_ptr<mtc::SerialContainer> MTCTaskNode::buildPickContainer(
    const std::string& side, const std::string& arm_group, const std::string& gripper_group,
    const std::string& gripper_joint_name, const std::string& eef_name, const std::string& hand_frame,
    const Eigen::Vector3d& grasp_rotation_axis, double grasp_rotation_angle, double grasp_z_offset,
    const std::shared_ptr<mtc::solvers::CartesianPath>& cartesian_planner,
    const std::shared_ptr<mtc::solvers::JointInterpolationPlanner>& interpolation_planner,
    mtc::Stage* current_state_ptr, mtc::Stage** attach_stage_out)
{
  auto grasp = std::make_unique<mtc::SerialContainer>(side + " pick stick");
  grasp->properties().set("group", arm_group);
  grasp->properties().set("eef", eef_name);
  grasp->properties().set("ik_frame", hand_frame);

  // approach
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>(side + " approach", cartesian_planner);
    stage->properties().set("marker_ns", side + "_approach");
    stage->setIKFrame(hand_frame);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.05, 0.10);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = hand_frame;
    vec.vector.z = 1.0;   // <-- CONFIRM per side, same as right's was confirmed -- left's hand
                            //     frame is not guaranteed to share the same native convention
    stage->setDirection(vec);
    grasp->insert(std::move(stage));
  }

  // grasp pose generation + IK
  {
    auto stage = std::make_unique<mtc::stages::GenerateGraspPose>(side + " generate grasp pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", side + "_grasp_pose");
    stage->setPreGraspPose("open");
    stage->setObject("stick");   // the SAME object both sides interact with
    stage->setAngleDelta(M_PI / 12);
    stage->setMonitoredStage(current_state_ptr);

    // No longer hardcoded -- each side now passes its own values in, so
    // tuning left never touches right's already-proven ones.
    Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
    grasp_frame_transform.linear() =
        Eigen::AngleAxisd(grasp_rotation_angle, grasp_rotation_axis).toRotationMatrix();
    grasp_frame_transform.translation().z() = grasp_z_offset;

    auto wrapper = std::make_unique<mtc::stages::ComputeIK>(side + " grasp pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(8);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame(grasp_frame_transform, hand_frame);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    grasp->insert(std::move(wrapper));
  }

  // allow collision with the stick while grasping
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(side + " allow collision");
    stage->allowCollisions("stick", gripper_group, true);
    grasp->insert(std::move(stage));
  }

  // close gripper
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>(side + " close hand", interpolation_planner);
    stage->setGroup(gripper_group);
    // FIXED: was setGoal("closed") -- fully shut (0.0) only works with an
    // empty gripper, which is what the "close hand (rest)" stages further
    // down correctly use it for. Partial closure sized to the bar instead.
    stage->setGoal(std::map<std::string, double>{ { gripper_joint_name, 0.025 } });
    grasp->insert(std::move(stage));
  }

  // attach stick to gripper
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(side + " attach");
    stage->attachObject("stick", hand_frame);
    *attach_stage_out = stage.get();
    grasp->insert(std::move(stage));
  }

  // lift
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>(side + " lift", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.05, 0.12);
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", side + "_lift");

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "openarm_body_link0";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    grasp->insert(std::move(stage));
  }

  return grasp;
}

std::unique_ptr<mtc::SerialContainer> MTCTaskNode::buildPlaceContainer(
    const std::string& side, const std::string& arm_group, const std::string& gripper_group,
    const std::string& eef_name, const std::string& hand_frame,
    double place_offset_x, double place_offset_y,
    const std::shared_ptr<mtc::solvers::CartesianPath>& cartesian_planner,
    const std::shared_ptr<mtc::solvers::JointInterpolationPlanner>& interpolation_planner,
    mtc::Stage* attach_stage)
{
  auto place = std::make_unique<mtc::SerialContainer>(side + " place stick");
  place->properties().set("group", arm_group);
  place->properties().set("eef", eef_name);
  place->properties().set("ik_frame", hand_frame);

  // place pose generation + IK
  {
    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>(side + " generate place pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", side + "_place_pose");
    stage->setObject("stick");

    // Offset is relative to the stick's CURRENT pose at this point in the
    // chain -- for left, that's wherever right actually left it, not the
    // stick's original pick location.
    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = "stick";
    target_pose_msg.pose.position.x = place_offset_x;
    target_pose_msg.pose.position.y = place_offset_y;
    target_pose_msg.pose.orientation.w = 1.0;
    stage->setPose(target_pose_msg);
    stage->setMonitoredStage(attach_stage);

    auto wrapper = std::make_unique<mtc::stages::ComputeIK>(side + " place pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(2);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame("stick");
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    place->insert(std::move(wrapper));
  }

  // open gripper
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>(side + " open (release)", interpolation_planner);
    stage->setGroup(gripper_group);
    stage->setGoal("open");
    place->insert(std::move(stage));
  }

  // forbid collision again
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(side + " forbid collision");
    stage->allowCollisions("stick", gripper_group, false);
    place->insert(std::move(stage));
  }

  // detach
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(side + " detach");
    stage->detachObject("stick", hand_frame);
    place->insert(std::move(stage));
  }

  // retreat
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>(side + " retreat", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.05, 0.12);
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", side + "_retreat");

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "openarm_body_link0";
    vec.vector.x = -0.15;
    stage->setDirection(vec);
    place->insert(std::move(stage));
  }

  return place;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("openarm handover: right places, left picks up and relocates");
  task.loadRobotModel(node_);

  const std::string right_arm = "right_arm", left_arm = "left_arm";
  const std::string right_gripper = "right_gripper", left_gripper = "left_gripper";
  const std::string right_hand = "openarm_right_hand", left_hand = "openarm_left_hand";
  const std::string right_ee = "right_ee", left_ee = "left_ee";
  
  const std::string right_finger_joint = "openarm_right_finger_joint1";
  const std::string left_finger_joint  = "openarm_left_finger_joint1";

  // Right: proven working on real hardware -- do not change while
  // debugging left.
  const Eigen::Vector3d RIGHT_GRASP_AXIS = Eigen::Vector3d::UnitY();
  const double RIGHT_GRASP_ANGLE = M_PI / 2;
  const double RIGHT_GRASP_Z_OFFSET = 0.08;

  // Left: still a fresh experiment -- CHANGE THESE, not right's, while
  // tuning. Confirmed 0/25 total IK failure with UnitY/+90/0.08, the same
  // pattern right had before its rotation was found empirically. Two
  // things to check, in order:
  //  1. Reachability first, independent of rotation: in RViz, drag
  //     left_arm's interactive marker to roughly where right actually
  //     placed the bar (watch the "right place stick" stage animate to
  //     see exactly where that is) and hit Plan. If that alone fails,
  //     the problem is reach/position, not rotation -- move the object
  //     closer to left before touching rotation at all.
  //  2. If reachable, try these rotations in order, watching the grasp
  //     markers each time: UnitX+90, UnitX-90, UnitY-90 (already have
  //     UnitY+90 confirmed failing). Keep Z_OFFSET at 0.08 to start --
  //     re-tune it only after a rotation succeeds at all.
  const Eigen::Vector3d LEFT_GRASP_AXIS = Eigen::Vector3d::UnitY();   // <-- try this first
  const double LEFT_GRASP_ANGLE = M_PI / 2;
  const double LEFT_GRASP_Z_OFFSET = 0.08;
  
  mtc::Stage* current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setMaxVelocityScalingFactor(0.3);
  sampling_planner->setMaxAccelerationScalingFactor(0.3);

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(0.3);
  interpolation_planner->setMaxAccelerationScalingFactor(0.3);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.3);
  cartesian_planner->setMaxAccelerationScalingFactor(0.3);
  cartesian_planner->setStepSize(.01);

  // =====================================================================
  // RIGHT ARM: pick the bar from its original spot, place it near
  // centerline (comment this whole block out to isolate/test left alone,
  // same as the bimanual node's block-isolation approach)
  // =====================================================================
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("right hands_up", interpolation_planner);
    stage->setGroup(right_arm);
    stage->setGoal("hands_up");
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("right open", sampling_planner);
    stage->setTimeout(5.0);
    stage->setGroup(right_gripper);
    stage->setGoal("open");
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "right move to pick",
        mtc::stages::Connect::GroupPlannerVector{ { right_arm, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  mtc::Stage* right_attach_stage = nullptr;
  task.add(buildPickContainer("right", right_arm, right_gripper, right_finger_joint,
  			       right_ee, right_hand,
                               RIGHT_GRASP_AXIS, RIGHT_GRASP_ANGLE, RIGHT_GRASP_Z_OFFSET,
                               cartesian_planner, interpolation_planner,
                               current_state_ptr, &right_attach_stage));

  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "right move to place",
        mtc::stages::Connect::GroupPlannerVector{ { right_arm, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  // Places the bar near the centerline (+x, +y from its original spot) --
  // within reach of the left arm for the handover. Placeholder, same as
  // every position in this project -- confirm in RViz.
  task.add(buildPlaceContainer("right", right_arm, right_gripper, right_ee, right_hand,
                                0.1, 0.3, cartesian_planner, interpolation_planner,
                                right_attach_stage));

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("right close hand (rest)", interpolation_planner);
    stage->setGroup(right_gripper);
    stage->setGoal("closed");
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("right return home", interpolation_planner);
    stage->setGroup(right_arm);
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  //New addition to store current state of right arm after it returns home, so left can use it as a reference for its pick
  mtc::Stage* right_return_home_ptr = nullptr;
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("right return home", interpolation_planner);
    stage->setGroup(right_arm);
    stage->setGoal("home");
    right_return_home_ptr = stage.get();   // store raw pointer
    task.add(std::move(stage));
  }

  // =====================================================================
  // LEFT ARM: pick up the SAME bar from wherever right left it, place it
  // somewhere else again (comment this block out to isolate/test right
  // alone)
  // =====================================================================

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("left hands_up", interpolation_planner);
    stage->setGroup(left_arm);
    stage->setGoal("hands_up");
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("left open", sampling_planner);
    stage->setTimeout(5.0);
    stage->setGroup(left_gripper);
    stage->setGoal("open");
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "left move to pick",
        mtc::stages::Connect::GroupPlannerVector{ { left_arm, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  mtc::Stage* left_attach_stage = nullptr;
  task.add(buildPickContainer("left", left_arm, left_gripper, left_finger_joint, 
  			       left_ee, left_hand,
                               LEFT_GRASP_AXIS, LEFT_GRASP_ANGLE, LEFT_GRASP_Z_OFFSET,
                               cartesian_planner, interpolation_planner,
                               right_return_home_ptr, &left_attach_stage));

  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "left move to place",
        mtc::stages::Connect::GroupPlannerVector{ { left_arm, sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  // Moves the bar further into left's own side -- "somewhere else again."
  // Placeholder, same as every position -- confirm in RViz.
  task.add(buildPlaceContainer("left", left_arm, left_gripper, left_ee, left_hand,
                                0.0, 0.6, cartesian_planner, interpolation_planner,
                                left_attach_stage));

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("left close hand (rest)", interpolation_planner);
    stage->setGroup(left_gripper);
    stage->setGoal("closed");
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("left return home", interpolation_planner);
    stage->setGroup(left_arm);
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

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
