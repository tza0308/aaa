#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

//用户定义的约束
//用户定义的约束也可以指定给PlanningScene类.这是通过使用setStateFeasibilityPredicate函数指定回调来完成的
//用于检查熊猫机器人的"panda_joint1"是处于正角还是负角.
bool stateFeasibilityTestExample(const robot_state::RobotState& kinematic_state, bool verbose)
{
  const double* joint_values = kinematic_state.getJointPositions("panda_joint1");
  return (joint_values[0] > 0.0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_kinematics");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::size_t count = 0;
  //设置
  //PlanningSceneMonitor是使用来自机器人关节和机器人传感器的数据创建和维护当前规划场景的推荐办法
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelConstPtr& kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);



  //碰撞检查
  //自碰撞检查
  //我们要做的第一件事是检查当前状态下的机器人是否处于自碰撞状态,即机器人当前的配置是否会导致机器人的部件相互碰撞
  //构造一个CollisionRequest对象和一个CollisionResult对象并将它们传递给碰撞检查函数
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request,collision_result);
  ROS_INFO_STREAM("Test 1: Current state is" << (collision_result.collision ? "in" : "not in"));
  //改变状态
  //更改机器人的当前状态.规划场景在内部保持当前状态.可以获得对它的引用并对其进行更改,
  //然后检查新机器人配置的碰撞.***需要在发出新的碰撞检查请求之前清除碰撞结果
  moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request,collision_result);
  ROS_INFO_STREAM("Test 2: Current state is" << (collision_result.collision ? "in" : "not in"));
  //检查组
  //将只对熊猫的手进行碰撞检查,即检查手与机器人身体其他部位之间是否有任何碰撞
  //通过将组名"hand"添加到碰撞请求中来专门要求这一点
  collision_request.group_name = "hand";
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 3: Current state is " << (collision_result.collision ? "in" : "not in"));
  //获取关节信息
  //手动将panda手臂设置到我们知道内部碰撞确实发生的位置
  //这个状态现在实际超出了panda的关节限制.也可以直接检查
  std::vector<double> jonit_values = {0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0};
  const moveit::core::JointModelGroup* jonit_model_group = current_state.getJointModelGroup("panda_arm");
  current_state.setJointGroupPositions(jonit_model_group, jonit_values);
  ROS_INFO_STREAM("Test 4: Current state is " << (current_state.satisfiesBounds(jonit_model_group) ? "valid" : "not valid"));
  //获取在panda手臂的给定配置下可能发生的任何碰撞的关节信息.
  //通过在碰撞请求中填写适当的字段并指定要返回的最大关节数量来请求关节信息
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request,collision_result);
  ROS_INFO_STREAM("Test 5: Current state is" << (collision_result.collision ? "in" : "not in"));
  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
  {
    ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
  }
  //修改允许的碰撞矩阵
  //AllowCollisionMatrix(ACM)提供了一种机制来告诉碰撞世界忽略某些对象之间的碰撞.机器人的两个部分和世界中的对象
  //我们可以告诉碰撞检查器忽略上面报告的链接之间的所有碰撞,即使链接实际上处于碰撞状态
  //碰撞检查器也会忽略这些碰撞并返回机器人的这种特定状态的非碰撞状态
  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  moveit::core::RobotState copied_state = planning_scene.getCurrentState();
  collision_detection::CollisionResult::ContactMap::const_iterator it2;
  for (it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2)
  {
    acm.setEntry(it->first.first, it->first.second, true);
  }
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);
  ROS_INFO_STREAM("Test 6: Current state is" << (collision_result.collision ? "in" : "not in"));
  //全碰撞检查
  //虽然一直在检查自碰撞,但可以使用checkcollision函数来检查自碰撞和与环境的碰撞
  //与环境的碰撞检查将使用机器人的填充版本.填充有助于使机器人远离环境中的障碍物
  collision_result.clear();
  planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
  ROS_INFO_STREAM("Test 7: Current state is" << (collision_result.collision ? "in" : "not in"));



 //约束检查
 //PlanningScene类还包括用于见车约束的易于使用的函数调用.约束可以有两种类型:
 //(a)从KinematicConstraint集合中选择的约束:即JointConstraint,PositionConstraint,OrientationConstraint和VisibilityConstraint
 //(b)通过回调指定的用户定义约束
 //检查运动学约束
 //首先在panda机器人的panda_arm组的末端执行器上定义一个简单的位置和方向约束.
 //请注意使用遍历函数来填充约束(在moveit_core中kinamatic_constraints目录的utils.h文件中找到)
 std::string end_effector_name = jonit_model_group->getLinkModelNames().back();
 geometry_msgs::PoseStamped desired_pose;
 desired_pose.pose.orientation.w = 1.0;
 desired_pose.pose.position.x = 0.3;
 desired_pose.pose.position.y = -0.185;
 desired_pose.pose.position.z = 0.5;
 desired_pose.header.frame_id = "panda_link0";
 moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(end_effector_name, desired_pose);
 //现在,使用PlanningScene类中的isStateConstrained函数检查该约束的状态
 copied_state.setToRandomPositions();
 copied_state.update();
 bool constrained = planning_scene.isStateConstrained(copied_state, goal_constraint);
 ROS_INFO_STREAM("Test 8: Random state is " << (constrained ? "constraind" : "not constrained"));
 //有一种更有效的检查约束的方法
 //首先构造一个KinematicConstraintSet,它预处理ROS约束消息并将其设置为快速处理
 kinematic_constraints::KinematicConstraintSet kinematic_constraint_set(kinematic_model);
 kinematic_constraint_set.add(goal_constraint, planning_scene.getTransforms());
 bool constrained_2 = planning_scene.isStateConstrained(copied_state, kinematic_constraint_set);
 ROS_INFO_STREAM("Test 9: Random state is " << (constrained_2 ? "constrained" : "not constrained"));
 //使用KinematicConstraintSet类可以直接执行此操作
 kinematic_constraints::ConstraintEvaluationResult constraint_eval_result = kinematic_constraint_set.decide(copied_state);
 ROS_INFO_STREAM("Test 10: Random state is " << (constraint_eval_result.satisfied ? "constrained" : "not constrained"));
 //无论何时调用isStateFeasible,都会调用此用户定义的回调
 planning_scene.setStateFeasibilityPredicate(stateFeasibilityTestExample);
 bool state_feasible = planning_scene.isStateFeasible(copied_state);
 ROS_INFO_STREAM("Test 11: Random state is " << (state_feasible ? "feasible" : "not feasible"));
 //每当调用isStateValid时.都会进行三项检查
 //1.碰撞检查 2.约束检查 3.使用用户自定义的回调进行可行性检查
 bool state_valid = planning_scene.isStateValid(copied_state, kinematic_constraint_set, "panda_arm");
 ROS_INFO_STREAM("Test 12: Random state is " << (state_valid ? "valid" : "not valid"));
 //注意.用过Moveit和ompl可用的所有规划器当前将使用用户自定义的回调执行碰撞检查,约束检查和可行性检查
 ros::shutdown();
 return 0;
}