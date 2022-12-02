#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "motionplanning_pipeline");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");
  //设置开始使用规划管道,在加载规划器之前.需要两个对象.
  //1.RobotModel  2.PlanningScene
  //首先实例化一个RobotModelLoader对象,该对象将在ROS参数服务器上查找机器人描述并构造一个RobotModel供我们使用
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
  //使用RobotModelLoader,可以构建一个平面场景监视器,将创建一个规划场景,监控规划场景差异,并将差异应用到它的内部规划场景
  //调用startSceneMonitor  startWorldGeometryMonitor  startStateMonitor来完全初始化规划场景监视器
  planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();
  psm->startStateMonitor();
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("gluon");
  //设置PlanningPipeline对象,它将使用ROS参数服务器来确定请求适配器集和要使用的规划插件
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
        new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));
  //可视化
  //MoveitVisualTools提供了许多用于在Rviz中可视化对象,机器人和轨迹的功能以及调试工具.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("dummy");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "dummy";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.4;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;
  //位置公差为0.01m,方向公差为0.01弧度
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);
  req.group_name = "gluon";
  moveit_msgs::Constraints pose_goal = 
    kinematic_constraints::constructGoalConstraints("6_Link", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);
  //在规划之前,需要在规划场景上设置一个只读锁,这样它就不会在规划时修改世界表示
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    planning_pipeline->generatePlan(lscene, req, res);
  }
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could  not compute plan successfully");
    return 0;
  }




  //可视化结果
  ros::Publisher display_publisher = 
    node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  //可视化轨迹
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  


  // //关节空间目标
  // //设置一个关节空间目标
  // moveit::core::RobotState goal_state(*robot_state);
  // std::vector<double> joint_values = {-1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0};
  // goal_state.setJointGroupPositions(joint_model_group, joint_values);
  // moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  // req.goal_constraints.clear();
  // req.goal_constraints.push_back(joint_goal);
  // //在规划之前,需要在规划场景上设置一个只读锁,这样它就不会在规划时修改世界表示
  // {
  //   planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
  //   planning_pipeline->generatePlan(lscene, req, res);
  // }
  // if (res.error_code_.val != res.error_code_.SUCCESS)
  // {
  //   ROS_ERROR("Could  not compute plan successfully");
  //   return 0;
  // }
  // ROS_INFO(" Visualizing the trajectory");
  // res.getMessage(response);
  // display_trajectory.trajectory_start = response.trajectory_start;
  // display_trajectory.trajectory.push_back(response.trajectory);
  //  //将看到两条规划轨迹串联
  // display_publisher.publish(display_trajectory);
  // visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  




  // //使用规划请求适配器
  // //规划请求适配器允许我们指定应该在规划发生之前或在结果路径上完成计划之后发生的一系列操作
  // robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  // robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  // //moveit::core::
  // //现在,将其中一个关节设置为稍微超出其上限
  // const moveit::core::JointModel* joint_model = joint_model_group->getJointModel("panda_joint3");
  // const moveit::core::JointModel::Bounds& joint_bounds = joint_model->getVariableBounds();
  // std::vector<double> tmp_values(1, 0.0);
  // tmp_values[0] =joint_bounds[0].min_position_ - 0.01;
  // robot_state->setJointPositions(joint_model, tmp_values);
  // req.goal_constraints.clear();
  // req.goal_constraints.push_back(pose_goal);
  // //在规划之前,需要在规划场景上设置一个只读锁,这样它就不会在规划时修改世界表示
  // {
  //   planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
  //   planning_pipeline->generatePlan(lscene, req, res);
  // }
  // if (res.error_code_.val != res.error_code_.SUCCESS)
  // {
  //   ROS_ERROR("Could  not compute plan successfully");
  //   return 0;
  // }
  // ROS_INFO(" Visualizing the trajectory");
  // res.getMessage(response);
  // display_trajectory.trajectory_start = response.trajectory_start;
  // display_trajectory.trajectory.push_back(response.trajectory);
  //  //将看到三条规划轨迹串联
  // display_publisher.publish(display_trajectory);
  // visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  // ROS_INFO("DONE");

  return 0;
}
