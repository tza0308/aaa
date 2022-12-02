#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planningscene_rosapi");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;
  //可视化
  //MoveItVisualTools提供了许多用在Rviz中可视化对象,机器人和轨迹的功能以及调试工具
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();


  //ROS API
  //规划场景发布者的ROS API是通过使用"diffs"的主题接口.规划场景差异是当前规划场景(由move_group节点维护)
  //与用户所需的新规划场景之间的差异
  //宣传所需的主题
  //创建一个发布者并等待订阅者
  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  //***************定义附加对象消息*****************
  //将使用此消息从世界中添加或减去对象.并将对象附加到机器人上
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "panda_hand";
  attached_object.object.header.frame_id = "panda_hand";
  attached_object.object.id = "box";
  geometry_msgs::Pose pose;
  pose.position.z = 0.11;
  pose.orientation.w = 1.0;
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.075;
  primitive.dimensions[1] = 0.075;
  primitive.dimensions[2] = 0.075;
  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);
  //将对象附加到机器人需要将相应的操作指定为ADD操作
  attached_object.object.operation = attached_object.object.ADD;
  //由于将对象附加到机器人手上以模拟拾取对象,因此希望碰撞检查器忽略对象与机器人手之间的碰撞;
  attached_object.touch_links = std::vector<std::string>{"panda_head", "panda_leftfinger", "panda_rightfinger"};



  //将对象添加到环境中
  //通过将对象添加到规划场景的"世界"部分中的碰撞对象集,将对象添加到环境中.
  //我们这里仅使用attatch_object消息的"object"字段
  ROS_INFO("Adding the object into the world at the location of the hand");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");



  //将对象附加到机器人
  //当机器人从环境中拾取一个物体时.需要将物体"附加"到机器人上,以便处理机器人模型的任何组件都知道要考虑附加的物体.例如碰撞检查
  //附加对象需要两个操作
  //1.从环境中移除原始对象
  //2.将物体连接到机器人
  moveit_msgs::CollisionObject remove_object;
  remove_object.id = "box";
  remove_object.header.frame_id = "panda_hand";
  remove_object.operation = remove_object.REMOVE;
  //注意如何通过首先清楚这些字段来确保差异消息不包含其他附加对象或冲突对象
  ROS_INFO("Attaching the object to the hand and removing it from the world");
  //planning_scene.robot_state.attached_collision_objects.clear();
  //planning_scene.robot_state.attached_collision_objects.push_back(remove_object);
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene.robot_state.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");



  //从机器人上拆下一个物体
  //从机器人上拆下一个物体需要两个操作
  //1.从机器人上拆下物体
  //2.将对象重新引入环境
  moveit_msgs::AttachedCollisionObject detach_object;
  detach_object.object.id = "box";
  detach_object.link_name = "panda_hand";
  detach_object.object.operation = attached_object.object.REMOVE;
  //请注意我们如何通过首先清除这些字段来确保差异消息不包含其他附加对象或冲突对象
  ROS_INFO("Detaching the object from the robot and returning it to the world");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene.robot_state.is_diff = true;
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");



  //从碰撞世界移除对象
  //从碰撞世界中移除对象只需要使用之前定义的移除对象消息
  ROS_INFO("Removing the object from the world");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene_diff_publisher.publish(planning_scene);
  ros::shutdown();
  return 0;
}