#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
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
  void setupPlanningScene();

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

void MTCTaskNode::setupPlanningScene()
{
  // A long, thin cylinder standing upright in front of the robot -- no
  // table. Position/height are placeholder estimates (matching the general
  // area earlier Cartesian tests reached successfully); watch where it
  // lands in RViz relative to the arm and adjust if it's out of reach.
  moveit_msgs::msg::CollisionObject stick;
  stick.id = "stick";
  stick.header.frame_id = "openarm_body_link0";   // confirmed real root link
  stick.primitives.resize(1);
  stick.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  // CYLINDER dimensions are [height, radius], in that order -- swap them
  // and you silently get a short fat disc instead of a long thin stick.
  stick.primitives[0].dimensions = { 0.8, 0.015 };   // 80cm long, 1.5cm radius

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.2;
  pose.position.y = -0.4;
  pose.position.z = 0.4;   // stick's own center height off the root frame
  pose.orientation.w = 1.0;  // upright (cylinder's local Z, its long axis, points straight up)
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
  task.stages()->setName("openarm stick pick place");
  task.loadRobotModel(node_);

  // ----- confirmed against your openarm_bimanual.srdf -----
  const auto& arm_group_name = "right_arm";
  const auto& hand_group_name = "right_gripper";   // planning group, for open/close MoveTo stages
  const auto& hand_frame = "openarm_right_hand";    // confirmed end-effector link
  const auto& eef_name = "right_ee";                // confirmed end_effector NAME (different from
                                                       //     the group name -- see the note this bit
                                                       //     us with earlier: "unknown end effector"
                                                       //     if you use hand_group_name here instead)
  const auto& root_frame = "openarm_body_link0";    // confirmed real root link

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", eef_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.3);
  cartesian_planner->setMaxAccelerationScalingFactor(0.3);
  cartesian_planner->setStepSize(.01);

  // ----- move away from the singular "home" pose first -----
  // "home" is all joints at zero -- a fully straightened 7-DOF arm, right
  // at a kinematic singularity. CartesianPath (used by approach/lift/
  // retreat below) needs the end-effector to move in a literal straight
  // line via the Jacobian, and that fails immediately from a singular
  // start (this is what caused the earlier 0.000000-achieved failures).
  // "hands_up" is the SRDF's existing non-singular state -- go there first
  // with joint interpolation (which doesn't care about singularities).
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move away from singularity", interpolation_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("hands_up");
    task.add(std::move(stage));
  }

  // ----- open gripper -----
  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  // ----- move to pick -----
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage = nullptr;

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick stick");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // approach
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach stick", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->setIKFrame(hand_frame);   // fixed: this was the missing piece -- "link" (below) isn't
                                         //     the property MoveRelative actually needs; setIKFrame
                                         //     (matching lift stick / retreat) is
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.10);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;   // best-guess "toward the object" direction for openarm_right_hand's
                              //     frame orientation -- watch the approach animate in RViz and
                              //     flip axis/sign if it goes the wrong way
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // grasp pose generation + IK
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("stick");
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);

      // IDENTITY rotation (what we had) applies no correction at all -- the
      // grasp ends up using openarm_right_hand's own native URDF
      // orientation, which is apparently "approach from directly above"
      // for every one of the 25 samples (they differ only by angle_delta,
      // a rotation AROUND the object, not a change in approach elevation).
      // Changing the "approach object" stage's direction vector (x/y/z)
      // does NOT fix this -- that only changes which way the hand
      // TRANSLATES on the final short approach, not which way it's FACING.
      // To get a side/horizontal approach instead of top-down, the fix is
      // a rotation here. We don't know openarm_right_hand's native axis
      // convention for certain -- rather than guess once and trust it (the
      // mistake that broke things with the borrowed Panda rotation
      // earlier), try these in order and watch the grasp markers in RViz
      // each time:
      //   1) UnitX(), +M_PI/2   (below)
      //   2) UnitX(), -M_PI/2
      //   3) UnitY(), +M_PI/2
      //   4) UnitY(), -M_PI/2
      // One of these four should visibly tip the approach from vertical to
      // horizontal; whichever does is the right one to keep.
      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      grasp_frame_transform.linear() =
          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()).toRotationMatrix();
      grasp_frame_transform.translation().z() = 0.15;  // this is the value that got you from 0/25 to
                                                          //     7/25 successful IK -- real evidence
                                                          //     this direction is closer to correct

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    // allow collision with the stick while grasping
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,stick)");
      stage->allowCollisions(
          "stick",
          task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
          true);
      grasp->insert(std::move(stage));
    }

    // close gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("closed");   // confirmed SRDF state name -- not "close"
      grasp->insert(std::move(stage));
    }

    // attach stick to gripper
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach stick");
      stage->attachObject("stick", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    // lift
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift stick", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.12);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = root_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  // ----- move to place -----
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                    { hand_group_name, sampling_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place stick");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // place pose generation + IK
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("stick");

      // "slightly different position" -- a small lateral shift from where
      // it was picked up, in the stick's own frame, not a full table's
      // width away like the original tutorial's 0.30.
      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "stick";
      target_pose_msg.pose.position.y = 0.15;   // <-- adjust: how far "elsewhere" should be
      target_pose_msg.pose.orientation.w = 1.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("stick");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    // open gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    // forbid collision again
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,stick)");
      stage->allowCollisions(
          "stick",
          task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
          false);
      place->insert(std::move(stage));
    }

    // detach
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach stick");
      stage->detachObject("stick", hand_frame);
      place->insert(std::move(stage));
    }

    // retreat
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.12);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = root_frame;
      vec.vector.x = -0.15;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  // ----- return home -----
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
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
