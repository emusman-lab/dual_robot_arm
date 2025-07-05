#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = -0.647;
  pose.position.y = -0.149;
  pose.position.z = 0.6;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
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
    RCLCPP_ERROR_STREAM(LOGGER, e);
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

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo dual task");
  task.loadRobotModel(node_);

  const auto& left_arm_group_name = "left_dobot_arm";
  const auto& left_hand_group_name = "left_dobot_end_effector";
  const auto& left_hand_frame = "left_base_link_gripper";

  const auto& right_arm_group_name = "right_dobot_arm";
  const auto& right_hand_group_name = "right_dobot_end_effector";
  const auto& right_hand_frame = "right_base_link_gripper";

  // Set task properties
  task.setProperty("left_group", left_arm_group_name);
  task.setProperty("left_eef", left_hand_group_name);
  task.setProperty("left_ik_frame", left_hand_frame);

  // Set task properties
  task.setProperty("right_group", right_arm_group_name);
  task.setProperty("right_eef", right_hand_group_name);
  task.setProperty("right_ik_frame", right_hand_frame);

  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // Define target pose
  // Define the joint targets
  std::map<std::string, double> left_joint_targets_stage_1_1 = 
  {
      {"left_joint1", -66.0/57.3},
      {"left_joint2", -21.0/57.3},
      {"left_joint3", -151.0/57.3},
      {"left_joint4", 86.0/57.3},
      {"left_joint5", -270.0/57.3},
      {"left_joint6", -59.0/57.3},
  };
  std::map<std::string, double> left_joint_targets_stage_final_1 = 
  {
      {"left_joint1", -66.0/57.3},
      {"left_joint2", -29.0/57.3},
      {"left_joint3", -151.0/57.3},
      {"left_joint4", 94.0/57.3},
      {"left_joint5", -270.0/57.3},
      {"left_joint6", -59.0/57.3},
  };
  std::map<std::string, double> left_joint_targets_stage_final_2 = 
  {
      {"left_joint1", -74.0/57.3},
      {"left_joint2", -36.0/57.3},
      {"left_joint3", -123.0/57.3},
      {"left_joint4", 73.0/57.3},
      {"left_joint5", -271.0/57.3},
      {"left_joint6", -71.0/57.3},
  };
  std::map<std::string, double> left_joint_targets_stage_place_1 = 
  {
      {"left_joint1", -34.0/57.3},
      {"left_joint2", -6.0/57.3},
      {"left_joint3", -110.0/57.3},
      {"left_joint4", 29.0/57.3},
      {"left_joint5", -272.0/57.3},
      {"left_joint6", -27.0/57.3},
  };
  std::map<std::string, double> left_joint_targets_stage_place_1_2 = 
  {
      {"left_joint1", -35.0/57.3},
      {"left_joint2", -35.0/57.3},
      {"left_joint3", -123.0/57.3},
      {"left_joint4", 72.0/57.3},
      {"left_joint5", -272.0/57.3},
      {"left_joint6", -28.0/57.3},
  };
  std::map<std::string, double> left_joint_targets_stage_place_2_2 = 
  {
      {"left_joint1", -36.0/57.3},
      {"left_joint2", -22.0/57.3},
      {"left_joint3", -119.0/57.3},
      {"left_joint4", 54.0/57.3},
      {"left_joint5", -272.0/57.3},
      {"left_joint6", -29.0/57.3},
  };



  ///////////////////////
  std::map<std::string, double> right_joint_targets_stage_1_1 = 
  {
      {"right_joint1", -66.0/57.3},
      {"right_joint2", -21.0/57.3},
      {"right_joint3", -151.0/57.3},
      {"right_joint4", 86.0/57.3},
      {"right_joint5", -270.0/57.3},
      {"right_joint6", -59.0/57.3},
  };
  std::map<std::string, double> right_joint_targets_stage_final_1 = 
  {
      {"right_joint1", -66.0/57.3},
      {"right_joint2", -29.0/57.3},
      {"right_joint3", -151.0/57.3},
      {"right_joint4", 94.0/57.3},
      {"right_joint5", -270.0/57.3},
      {"right_joint6", -59.0/57.3},
  };
  std::map<std::string, double> right_joint_targets_stage_place_1 = 
  {
      {"right_joint1", -115.0/57.3},
      {"right_joint2", -16.0/57.3},
      {"right_joint3", -112.0/57.3},
      {"right_joint4", 41.0/57.3},
      {"right_joint5", -267.0/57.3},
      {"right_joint6", -108.0/57.3},
  };
  std::map<std::string, double> right_joint_targets_stage_place_1_2 = 
  {
      {"right_joint1", -115.0/57.3},
      {"right_joint2", -27.0/57.3},
      {"right_joint3", -119.0/57.3},
      {"right_joint4", 60.0/57.3},
      {"right_joint5", -268.0/57.3},
      {"right_joint6", -107.0/57.3},
  };
  std::map<std::string, double> right_joint_targets_stage_final_2 = 
  {
      {"right_joint1", -73.0/57.3},
      {"right_joint2", -36.0/57.3},
      {"right_joint3", -123.0/57.3},
      {"right_joint4", 71.0/57.3},
      {"right_joint5", -272.0/57.3},
      {"right_joint6", -72.0/57.3},
  };
  std::map<std::string, double> right_joint_targets_stage_place_2_2 = 
  {
      {"right_joint1", -116.0/57.3},
      {"right_joint2", -18.0/57.3},
      {"right_joint3", -113.0/57.3},
      {"right_joint4", 44.0/57.3},
      {"right_joint5", -267.0/57.3},
      {"right_joint6", -109.0/57.3},
  };

  // LEFT: HAND STAGES
  auto left_stage_open_hand_1 = std::make_unique<mtc::stages::MoveTo>("LEFT: left_stage_open_hand_1", interpolation_planner);
  left_stage_open_hand_1->setGroup(left_hand_group_name);
  left_stage_open_hand_1->setGoal("open");
  

  auto set_left_joint_targets_stage_1_1 = std::make_unique<mtc::stages::MoveTo>("LEFT: set_left_joint_targets_stage_1_1", sampling_planner);
  set_left_joint_targets_stage_1_1->setGroup(left_arm_group_name);
  set_left_joint_targets_stage_1_1->setGoal(left_joint_targets_stage_1_1);
  

  auto set_left_joint_targets_stage_final_1 = std::make_unique<mtc::stages::MoveTo>("LEFT: set_left_joint_targets_stage_final_1", sampling_planner);
  set_left_joint_targets_stage_final_1->setGroup(left_arm_group_name);
  set_left_joint_targets_stage_final_1->setGoal(left_joint_targets_stage_final_1);
  

  auto left_stage_close_hand_1 = std::make_unique<mtc::stages::MoveTo>("LEFT: left_stage_close_hand_1", interpolation_planner);
  left_stage_close_hand_1->setGroup(left_hand_group_name);
  left_stage_close_hand_1->setGoal("close");
  

  auto set_left_joint_targets_stage_place_1 = std::make_unique<mtc::stages::MoveTo>("LEFT: left_joint_targets_stage_place_1", sampling_planner);
  set_left_joint_targets_stage_place_1->setGroup(left_arm_group_name);
  set_left_joint_targets_stage_place_1->setGoal(left_joint_targets_stage_place_1);

  auto set_left_joint_targets_stage_place_1_2 = std::make_unique<mtc::stages::MoveTo>("LEFT: left_joint_targets_stage_place_1_2", sampling_planner);
  set_left_joint_targets_stage_place_1_2->setGroup(left_arm_group_name);
  set_left_joint_targets_stage_place_1_2->setGoal(left_joint_targets_stage_place_1_2);
  

  auto left_stage_open_hand_2 = std::make_unique<mtc::stages::MoveTo>("LEFT: left_stage_open_hand_2", interpolation_planner);
  left_stage_open_hand_2->setGroup(left_hand_group_name);
  left_stage_open_hand_2->setGoal("open");
  

  auto set_go_to_home_position_1 = std::make_unique<mtc::stages::MoveTo>("LEFT: go_to_home_position_1", sampling_planner);
  set_go_to_home_position_1->setGroup(left_arm_group_name);
  set_go_to_home_position_1->setGoal("isaac_demo_29");
  

  auto set_left_joint_targets_stage_final_2 = std::make_unique<mtc::stages::MoveTo>("LEFT: set_left_joint_targets_stage_final_2", sampling_planner);
  set_left_joint_targets_stage_final_2->setGroup(left_arm_group_name);
  set_left_joint_targets_stage_final_2->setGoal(left_joint_targets_stage_final_2);
  

  auto left_stage_close_hand_2 = std::make_unique<mtc::stages::MoveTo>("LEFT: left_stage_close_hand_2", interpolation_planner);
  left_stage_close_hand_2->setGroup(left_hand_group_name);
  left_stage_close_hand_2->setGoal("close");
  

  auto set_left_joint_targets_stage_place_2 = std::make_unique<mtc::stages::MoveTo>("LEFT: left_joint_targets_stage_place_2", sampling_planner);
  set_left_joint_targets_stage_place_2->setGroup(left_arm_group_name);
  set_left_joint_targets_stage_place_2->setGoal(left_joint_targets_stage_place_1);

  auto set_left_joint_targets_stage_place_2_2 = std::make_unique<mtc::stages::MoveTo>("LEFT: left_joint_targets_stage_place_2_2", sampling_planner);
  set_left_joint_targets_stage_place_2_2->setGroup(left_arm_group_name);
  set_left_joint_targets_stage_place_2_2->setGoal(left_joint_targets_stage_place_2_2);
  

  auto left_stage_open_hand_3 = std::make_unique<mtc::stages::MoveTo>("LEFT: left_stage_open_hand_3", interpolation_planner);
  left_stage_open_hand_3->setGroup(left_hand_group_name);
  left_stage_open_hand_3->setGoal("open");
  

  auto set_go_to_home_position_2 = std::make_unique<mtc::stages::MoveTo>("LEFT: go_to_home_position_2", sampling_planner);
  set_go_to_home_position_2->setGroup(left_arm_group_name);
  set_go_to_home_position_2->setGoal("isaac_demo_29");
    

  // RIGHT: HAND STAGES

  auto right_stage_open_hand_1 = std::make_unique<mtc::stages::MoveTo>("RIGHT: right_stage_open_hand_1", interpolation_planner);
  right_stage_open_hand_1->setGroup(right_hand_group_name);
  right_stage_open_hand_1->setGoal("open");

  auto set_right_joint_targets_stage_1_1 = std::make_unique<mtc::stages::MoveTo>("RIGHT: set_right_joint_targets_stage_1_1", sampling_planner);
  set_right_joint_targets_stage_1_1->setGroup(right_arm_group_name);
  set_right_joint_targets_stage_1_1->setGoal(right_joint_targets_stage_1_1);
  
  auto set_right_joint_targets_stage_final_1 = std::make_unique<mtc::stages::MoveTo>("RIGHT: set_right_joint_targets_stage_final_1", sampling_planner);
  set_right_joint_targets_stage_final_1->setGroup(right_arm_group_name);
  set_right_joint_targets_stage_final_1->setGoal(right_joint_targets_stage_final_1);

  auto right_stage_close_hand_1 = std::make_unique<mtc::stages::MoveTo>("RIGHT: right_stage_close_hand_1", interpolation_planner);
  right_stage_close_hand_1->setGroup(right_hand_group_name);
  right_stage_close_hand_1->setGoal("close");
  
  auto set_right_joint_targets_stage_place_1 = std::make_unique<mtc::stages::MoveTo>("RIGHT: right_joint_targets_stage_place_1", sampling_planner);
  set_right_joint_targets_stage_place_1->setGroup(right_arm_group_name);
  set_right_joint_targets_stage_place_1->setGoal(right_joint_targets_stage_place_1);

  auto set_right_joint_targets_stage_place_1_2 = std::make_unique<mtc::stages::MoveTo>("RIGHT: right_joint_targets_stage_place_1_2", sampling_planner);
  set_right_joint_targets_stage_place_1_2->setGroup(right_arm_group_name);
  set_right_joint_targets_stage_place_1_2->setGoal(right_joint_targets_stage_place_1_2);

  auto right_stage_open_hand_2 = std::make_unique<mtc::stages::MoveTo>("RIGHT: right_stage_open_hand_2", interpolation_planner);
  right_stage_open_hand_2->setGroup(right_hand_group_name);
  right_stage_open_hand_2->setGoal("open");

  auto set_go_to_home_position_3 = std::make_unique<mtc::stages::MoveTo>("RIGHT: go_to_home_position_3", sampling_planner);
  set_go_to_home_position_3->setGroup(right_arm_group_name);
  set_go_to_home_position_3->setGoal("isaac_demo_29");

  auto set_right_joint_targets_stage_final_2 = std::make_unique<mtc::stages::MoveTo>("RIGHT: right_joint_targets_stage_final_2", sampling_planner);
  set_right_joint_targets_stage_final_2->setGroup(right_arm_group_name);
  set_right_joint_targets_stage_final_2->setGoal(right_joint_targets_stage_final_2);

  auto right_stage_close_hand_2 = std::make_unique<mtc::stages::MoveTo>("RIGHT: right_stage_close_hand_2", interpolation_planner);
  right_stage_close_hand_2->setGroup(right_hand_group_name);
  right_stage_close_hand_2->setGoal("close");

  auto set_right_joint_targets_stage_place_2 = std::make_unique<mtc::stages::MoveTo>("RIGHT: right_joint_targets_stage_place_2", sampling_planner);
  set_right_joint_targets_stage_place_2->setGroup(right_arm_group_name);
  set_right_joint_targets_stage_place_2->setGoal(right_joint_targets_stage_place_1);

  auto set_right_joint_targets_stage_place_2_2 = std::make_unique<mtc::stages::MoveTo>("RIGHT: right_joint_targets_stage_place_2_2", sampling_planner);
  set_right_joint_targets_stage_place_2_2->setGroup(right_arm_group_name);
  set_right_joint_targets_stage_place_2_2->setGoal(right_joint_targets_stage_place_2_2);

  auto right_stage_open_hand_3 = std::make_unique<mtc::stages::MoveTo>("RIGHT: right_stage_open_hand_3", interpolation_planner);
  right_stage_open_hand_3->setGroup(right_hand_group_name);
  right_stage_open_hand_3->setGoal("open");

  auto set_go_to_home_position_4 = std::make_unique<mtc::stages::MoveTo>("RIGHT: go_to_home_position_4", sampling_planner);
  set_go_to_home_position_4->setGroup(right_arm_group_name);
  set_go_to_home_position_4->setGoal("isaac_demo_29");

  // LEFT HAND CUBE 1
  task.add(std::move(stage_state_current));
  task.add(std::move(left_stage_open_hand_1));
  task.add(std::move(set_left_joint_targets_stage_1_1));
  task.add(std::move(set_left_joint_targets_stage_final_1));
  task.add(std::move(left_stage_close_hand_1));
  task.add(std::move(set_left_joint_targets_stage_place_1));
  task.add(std::move(set_left_joint_targets_stage_place_1_2));
  task.add(std::move(left_stage_open_hand_2));
  task.add(std::move(set_go_to_home_position_1));

  // // RIGHT HAND CUBE 1
  task.add(std::move(right_stage_open_hand_1));
  task.add(std::move(set_right_joint_targets_stage_1_1));
  task.add(std::move(set_right_joint_targets_stage_final_1));
  task.add(std::move(right_stage_close_hand_1));
  task.add(std::move(set_right_joint_targets_stage_place_1));
  task.add(std::move(set_right_joint_targets_stage_place_1_2));
  task.add(std::move(right_stage_open_hand_2));
  task.add(std::move(set_go_to_home_position_3));

  // // LEFT HAND CUBE 2
  task.add(std::move(set_left_joint_targets_stage_final_2));
  task.add(std::move(left_stage_close_hand_2));
  task.add(std::move(set_left_joint_targets_stage_place_2));
  task.add(std::move(set_left_joint_targets_stage_place_2_2));
  task.add(std::move(left_stage_open_hand_3));
  task.add(std::move(set_go_to_home_position_2));

  // // RIGHT HAND CUBE 2
  task.add(std::move(set_right_joint_targets_stage_final_2));
  task.add(std::move(right_stage_close_hand_2));
  task.add(std::move(set_right_joint_targets_stage_place_2));
  task.add(std::move(set_right_joint_targets_stage_place_2_2));
  task.add(std::move(right_stage_open_hand_3));
  task.add(std::move(set_go_to_home_position_4));


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

  // mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}