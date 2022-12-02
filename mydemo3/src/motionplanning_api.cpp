#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <boost/scoped_ptr.hpp>

int main(int argc, char** argv)
{
  const std::string node_name = "motionplanning_api";
  ros::init(argc, argv, node_name);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");
  //开始
  //规划器在moveit中设置为插件,可以使用ROS pluginlib接口加载想要使用的任何规划器.
  //在加载规划器之前.需要两个对象,一个RobotModel和一个PlanningScene
  //首先实例化一个RobotModelLoader对象.该对象将在ROS参数服务器上查找机器人描述并构造一个RObotModel供我们使用
  const std::string PLANNING_GROUP = "gluon";


  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  
  
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
  //使用RobotModel.构建一个维护世界状态的PlanningScene
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  
  
  //配置有效的机器人状态
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");
  //**************我们现在将构建一个加载器来按名称加载一个规划器**************
  //**************这里使用的是ROS pluginlib库*****************
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;
  //从ROS参数服务器获取想要加载的计划插件的名称,然后加载规划器以确保捕获所有异常
  if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader" << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
        ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (const auto& cls:classes)
    ss << cls << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }
  



  //可视化
  //MoveitVisualTools提供了许多用于在Rviz中可视化对象,机器人和轨迹的功能以及调试工具
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("dummy");
  visual_tools.loadRobotStatePub("/display_robot_state");
  visual_tools.enableBatchPublishing();

  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  visual_tools.loadRemoteControl();
  
  
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Montion Planning API Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  
  //姿势目标
  //将为panda的手臂创建一个运动规划请求,指定末端执行器的所需姿势作为输入
  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools.trigger();
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "dummy";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.2;
  pose.pose.position.z = 0.2;
  pose.pose.orientation.w = 1.0;
  //位置公差为0.01m,方向公差为0.01弧度
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);
  //使用kinematic_constraints包中提供的辅助函数将请求创建为约束.
  moveit_msgs::Constraints pose_goal = 
    kinematic_constraints::constructGoalConstraints("6_Link", pose, tolerance_pose, tolerance_angle);
  req.group_name = PLANNING_GROUP;
  req.goal_constraints.push_back(pose_goal);
  //现在构建一个封装场景,请求和响应的规划上下文.使用这个规划上下文调用规划器
  planning_interface::PlanningContextPtr context = 
    planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }




  //可视化结果
  // ros::Publisher display_publisher = 
  //   node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  //可视化轨迹
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  

  
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  
  
  
  visual_tools.trigger();
 // display_publisher.publish(display_trajectory);
  //将规划场景中的状态设置为上一个规划的最终状态
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());
  //显示目标状态
  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  // visual_tools.publishAxisLabeled(pose.pose, "goal_1");
  // visual_tools.publishText(text_pose, "Pose Goal (1)", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  

  // //关节空间目标
  // //设置一个关节空间目标
  // moveit::core::RobotState goal_state(robot_model);
  // std::vector<double> joint_values = {-1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0};
  // goal_state.setJointGroupPositions(joint_model_group, joint_values);
  // moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  // req.goal_constraints.clear();
  // req.goal_constraints.push_back(joint_goal);
  // //调用规划器并可视化轨迹
  // //重构规划内容
  // context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  // //响应规划
  // context->solve(res);
  // //检查规划是否成功
  // if (res.error_code_.val != res.error_code_.SUCCESS)
  // {
  //   ROS_ERROR("could not compute plan successfully");
  //   return 0;
  // }
  // //可视化轨迹
  // res.getMessage(response);
  // display_trajectory.trajectory.push_back(response.trajectory);
  // //将看到两条规划轨迹串联
  // visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  // visual_tools.trigger();
  // display_publisher.publish(display_trajectory);
  // //我们将添加更多目标。 但首先，将规划场景中的状态设置为上一个规划的最终状态
  // robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  // planning_scene->setCurrentState(*robot_state.get());
  // //显示目标状态
  // visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  // visual_tools.publishAxisLabeled(pose.pose, "goal_2");
  // visual_tools.publishText(text_pose, "Joint Space Goal (2)", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  // //现在，我们回到第一个目标，为定向约束规划做准备 
  // req.goal_constraints.clear();
  // req.goal_constraints.push_back(pose_goal);
  // context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  // context->solve(res);
  // res.getMessage(response);
  // display_trajectory.trajectory.push_back(response.trajectory);
  // visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  // visual_tools.trigger();
  // display_publisher.publish(display_trajectory);
  // //将规划场景中的状态设置为上一个规划的最终状态
  // robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  // planning_scene->setCurrentState(*robot_state.get());
  // //显示目标状态
  // visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  


  // //添加路径约束
  // //再次添加一个新的姿势目标.还将为运动添加路径约束
  // pose.pose.position.x = 0.32;
  // pose.pose.position.y = -0.25;
  // pose.pose.position.z = 0.65;
  // pose.pose.orientation.w = 1.0;
  // moveit_msgs::Constraints pose_goal_2 = 
  //   kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);
  // //尝试移动到新的目标位置
  // req.goal_constraints.clear();
  // req.goal_constraints.push_back(pose_goal_2);
  // //添加一条路径约束,访问末端执行器的水平
  // geometry_msgs::QuaternionStamped quaternion;
  // quaternion.header.frame_id = "panda_link0";
  // quaternion.quaternion.w = 1.0;
  // req.path_constraints = kinematic_constraints::constructGoalConstraints("panda_link8", quaternion);
  // //施加路径约束需要规划器在末端执行器的可能位置空间中进行推理.因此,还需要为允许的规划体积指定一个界限
  // //默认工作空间边界请求适配器(OMPL管道的一部分)自动填充.我们使用一个绝对包括手臂可达空间界限
  // req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
  // req.workspace_parameters.min_corner.z = -2.0;
  // req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = 
  // req.workspace_parameters.max_corner.z = 2.0;
  // //调用规划者并可视化到目前为止创建的所有规划
  // context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  // context->solve(res);
  // res.getMessage(response);
  // display_trajectory.trajectory.push_back(response.trajectory);
  // visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  // visual_tools.trigger();
  // display_publisher.publish(display_trajectory);
  // //将规划场景中的状态设置为上一个规划的最终状态
  // robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  // planning_scene->setCurrentState(*robot_state.get());
  // //显示目标状态
  // visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  // visual_tools.publishAxisLabeled(pose.pose, "goal_3");
  // visual_tools.publishText(text_pose, "Orientation Constrained Motion Plan (3)", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();






  return 0;
}