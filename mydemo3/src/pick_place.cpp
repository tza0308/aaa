#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  //添加熊猫机器人的两个手指关节
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";
  //将它们设置为打开，宽度足以容纳对象。
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);

}



void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  //添加熊猫机器人的两个手指关节
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  //将它们设置为关闭
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}



void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    //创建环境
    //创建向量以容纳3个碰撞对象
    std::vector<moveit_msgs::CollisionObject> collision_object;
    collision_object.resize(3);
    //添加最初保存多维数据集的第一个表
    collision_object[0].id = "table1";
    collision_object[0].header.frame_id = "panda_link0";
    //定义桌子的大小与位置
    collision_object[0].primitives.resize(1);
    collision_object[0].primitives[0].type = collision_object[0].primitives[0].BOX;
    collision_object[0].primitives[0].dimensions.resize(3);
    collision_object[0].primitives[0].dimensions[0] = 0.2;
    collision_object[0].primitives[0].dimensions[1] = 0.4;
    collision_object[0].primitives[0].dimensions[2] = 0.4;
    collision_object[0].primitive_poses.resize(1);
    collision_object[0].primitive_poses[0].position.x = 0.5;
    collision_object[0].primitive_poses[0].position.y = 0;
    collision_object[0].primitive_poses[0].position.z = 0.2;
    collision_object[0].primitive_poses[0].orientation.w = 1.0;


    //添加最初保存多维数据集的第二个表
    collision_object[1].id = "table2";
    collision_object[1].header.frame_id = "panda_link0";
    //定义桌子的大小与位置
    collision_object[1].primitives.resize(1);
    collision_object[1].primitives[0].type = collision_object[1].primitives[0].BOX;
    collision_object[1].primitives[0].dimensions.resize(3);
    collision_object[1].primitives[0].dimensions[0] = 0.4;
    collision_object[1].primitives[0].dimensions[1] = 0.2;
    collision_object[1].primitives[0].dimensions[2] = 0.4;
    collision_object[1].primitive_poses.resize(1);
    collision_object[1].primitive_poses[0].position.x = 0;
    collision_object[1].primitive_poses[0].position.y = 0.5;
    collision_object[1].primitive_poses[0].position.z = 0.2;
    collision_object[1].primitive_poses[0].orientation.w = 1.0;



    //定义要操作的对象
    collision_object[2].header.frame_id = "panda_link0";
    collision_object[2].id = "object";
     //定义对象的大小与位置
    collision_object[2].primitives.resize(1);
    collision_object[2].primitives[0].type = collision_object[1].primitives[0].BOX;
    collision_object[2].primitives[0].dimensions.resize(3);
    collision_object[2].primitives[0].dimensions[0] = 0.02;
    collision_object[2].primitives[0].dimensions[1] = 0.02;
    collision_object[2].primitives[0].dimensions[2] = 0.2;
    collision_object[2].primitive_poses.resize(1);
    collision_object[2].primitive_poses[0].position.x = 0.5;
    collision_object[2].primitive_poses[0].position.y = 0;
    collision_object[2].primitive_poses[0].position.z = 0.5;
    collision_object[2].primitive_poses[0].orientation.w = 1.0;


 
    collision_object[2].operation = collision_object[2].ADD;
    planning_scene_interface.applyCollisionObjects(collision_object);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    //选择管道
    //创建要尝试的抓取向量,当前仅创建单个抓取,这在使用抓取生成器生产和测试多个抓取时非常有用
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);
    //设置抓握姿势
    //这个是panda_link8的姿势
    //确保在设置抓握姿势时,将其设置为操纵器中最后一个链接的姿势,在本例中为"panda_link8"
    //必须补偿从"panda_link8"到手掌的变换末端执行器
    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI/2, -M_PI / 4, -M_PI / 2);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.415;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.5;
    
    //设置预掌握方法
    //相对于 frame_id 定义
    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    //方向设置为正 x 轴
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    //设置抓后撤退
    //相对于 frame_id 定义
    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
    //方向设置为正z轴
    grasps[0].post_grasp_retreat.direction.vector.x = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    //抓握前设置末端执行器的姿势
    openGripper(grasps[0].pre_grasp_posture);
    //抓握时设置末端执行器的姿势
    closedGripper(grasps[0].grasp_posture);
    //设置支撑面为table1
    move_group.setSupportSurfaceName("table1");
    //调用pick以使用给定的抓握拾取对象
    move_group.pick("object", grasps);
}



void place(moveit::planning_interface::MoveGroupInterface& group)
{
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  //设置地点位置姿势
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  //放置时它是对象中心的确切位置
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;
  //相对于 frame_id 定义
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  //方向设置为负z轴
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  //相对于 frame_id 定义
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  //方向设置为负y轴
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  openGripper(place_location[0].post_place_posture);
  group.setSupportSurfaceName("table2");
  group.place("object", place_location);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  group.setPlanningTime(45.0);

  addCollisionObjects(planning_scene_interface);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  pick(group);

  ros::WallDuration(1.0).sleep();

  place(group);

  ros::waitForShutdown();
  return 0;
}