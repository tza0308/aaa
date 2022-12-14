
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/scoped_ptr.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>


int main(int argc, char** argv)
{
  const std::string node_name = "tzademo";
  ros::init(argc, argv, node_name);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  // BEGIN_TUTORIAL
  // Start
  // ^^^^^
  // Setting up to start using a planner is pretty easy. Planners are
  // setup as plugins in MoveIt and you can use the ROS pluginlib
  // interface to load any planner that you want to use. Before we
  // can load the planner, we need two objects, a RobotModel and a
  // PlanningScene. We will start by instantiating a `RobotModelLoader`_
  // object, which will look up the robot description on the ROS
  // parameter server and construct a :moveit_core:`RobotModel` for us
  // to use.
  //
  // .. _RobotModelLoader:
  //     http://docs.ros.org/noetic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  const std::string PLANNING_GROUP = "gluon";

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);


  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  
  const moveit::core::JointModelGroup* joint_model_group = 
    move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  
  // //const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  // // Using the :moveit_core:`RobotModel`, we can construct a :planning_scene:`PlanningScene`
  // // that maintains the state of the world (including the robot).
  // planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  // // Configure a valid robot state
  // planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

  // // We will now construct a loader to load a planner, by name.
  // // Note that we are using the ROS pluginlib library here.
  // boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  // planning_interface::PlannerManagerPtr planner_instance;
  // std::string planner_plugin_name;

  // // We will get the name of planning plugin we want to load
  // // from the ROS parameter server, and then load the planner
  // // making sure to catch all exceptions.
  // if (!node_handle.getParam("planning_plugin", planner_plugin_name))
  //   ROS_FATAL_STREAM("Could not find planner plugin name");
  // try
  // {
  //   planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
  //       "moveit_core", "planning_interface::PlannerManager"));
  // }
  // catch (pluginlib::PluginlibException& ex)
  // {
  //   ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  // }
  // try
  // {
  //   planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
  //   if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
  //     ROS_FATAL_STREAM("Could not initialize planner instance");
  //   ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  // }
  // catch (pluginlib::PluginlibException& ex)
  // {
  //   const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
  //   std::stringstream ss;
  //   for (const auto& cls : classes)
  //     ss << cls << " ";
  //   ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
  //                                                        << "Available plugins: " << ss.str());
  // }

  // // Visualization
  // // ^^^^^^^^^^^^^
  // // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  // namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools("dummy");
  // visual_tools.loadRobotStatePub("/display_robot_state");
  // visual_tools.enableBatchPublishing();
  // visual_tools.deleteAllMarkers();  // clear all old markers
  // visual_tools.trigger();

  // /* Remote control is an introspection tool that allows users to step through a high level script
  //    via buttons and keyboard shortcuts in RViz */
  // visual_tools.loadRemoteControl();

  // /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
  // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  // text_pose.translation().z() = 1.75;
  // visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);

  // /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
  // visual_tools.trigger();

  // /* We can also use visual_tools to wait for user input */
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // // Pose Goal
  // // ^^^^^^^^^
  // // We will now create a motion plan request for the arm of the Panda
  // // specifying the desired pose of the end-effector as input.
  // visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  // visual_tools.trigger();
  // planning_interface::MotionPlanRequest req;
  // //planning_interface::MotionPlanResponse res;
  // planning_interface::MotionPlanDetailedResponse res;
  // geometry_msgs::PoseStamped pose;
  // pose.header.frame_id = "dummy";
  // pose.pose.position.x = 0.1;
  // pose.pose.position.y = 0.1;
  // pose.pose.position.z = 0.3;
  // pose.pose.orientation.w = 1.0;

  // // A tolerance of 0.01 m is specified in position
  // // and 0.01 radians in orientation
  // std::vector<double> tolerance_pose(3, 0.01);
  // std::vector<double> tolerance_angle(3, 0.01);

  // // We will create the request as a constraint using a helper function available
  // // from the
  // // `kinematic_constraints`_
  // // package.
  // //
  // // .. _kinematic_constraints:
  // //     http://docs.ros.org/noetic/api/moveit_core/html/cpp/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
  // moveit_msgs::Constraints pose_goal =
  //     kinematic_constraints::constructGoalConstraints("6_Link", pose, tolerance_pose, tolerance_angle);

  // req.group_name = PLANNING_GROUP;
  // req.goal_constraints.push_back(pose_goal);

  // // We now construct a planning context that encapsulate the scene,
  // // the request and the response. We call the planner using this
  // // planning context
  // planning_interface::PlanningContextPtr context =
  //     planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  // context->solve(res);
  // if (res.error_code_.val != res.error_code_.SUCCESS)
  // {
  //   ROS_ERROR("Could not compute plan successfully");
  //   return 0;
  // }

  // /* Visualize the trajectory */
  // //moveit_msgs::MotionPlanResponse response;
  // moveit_msgs::MotionPlanDetailedResponse response;
  // res.getMessage(response);

  // move_group_interface.execute(response.trajectory);

  // // END_TUTORIAL
  // /* Wait for user input */
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to exit the demo");
  // planner_instance.reset();

  return 0;
}

