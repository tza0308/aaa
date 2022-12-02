#include <ros/ros.h>
#include "interactivity/interactive_robot.h"
#include "interactivity/pose_string.h"
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection/collision_tools.h>


planning_scene::PlanningScene* g_planning_scene = 0;
shapes::ShapePtr g_world_cube_shape;
ros::Publisher* g_marker_array_publisher = 0;
visualization_msgs::MarkerArray g_collision_points;



void publishMarkers(visualization_msgs::MarkerArray& markers)
{
  // delete old markers
  if (g_collision_points.markers.size())
  {
    for (int i = 0; i < g_collision_points.markers.size(); i++)
      g_collision_points.markers[i].action = visualization_msgs::Marker::DELETE;

    g_marker_array_publisher->publish(g_collision_points);
  }

  // move new markers into g_collision_points
  std::swap(g_collision_points.markers, markers.markers);

  // draw new markers (if there are any)
  if (g_collision_points.markers.size())
    g_marker_array_publisher->publish(g_collision_points);
}









int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualizing_collisions_tutorial");
  ros::NodeHandle nh;
  //初始化规划场景和标记
  //在本教程中,我们使用interactiveRobot对象作为包装器,它结合了robot_model与立方体和交互式标记
  //我们还创建了一个PlanningScene用于碰撞检查
  InteractiveRobot robot;
  g_planning_scene = new planning_scene::PlanningScene(robot.robotModel());
  //将几何图形添加到PlanningScene
  Eigen::Isometry3d world_cube_pose;
  double world_cube_size;
  robot.getWorldGeometry(world_cube_pose, world_cube_size);
  g_world_cube_shape.reset(new shapes::Box(world_cube_size, world_cube_size, world_cube_size));
  g_planning_scene->getWorldNonConst()->addToObject("world_cube", g_world_cube_shape, world_cube_pose);



  //碰撞请求
  //为机器人创建一个碰撞请求
  collision_detection::CollisionRequest c_req;
  collision_detection::CollisionResult c_res;
  c_req.group_name = robot.getGroupName();
  c_req.contacts = true;
  c_req.max_contacts = 100;
  c_req.max_contacts_per_pair = 5;
  c_req.verbose = false;


  //检查碰撞
  //检查机器人与自身或世界之间的碰撞
  g_planning_scene->checkCollision(c_req, c_res, *robot.robotState());




    if (c_res.collision)
  {
    ROS_INFO("COLLIDING contact_point_count=%d", (int)c_res.contact_count);
    if (c_res.contact_count > 0)
    {
      std_msgs::ColorRGBA color;
      color.r = 1.0;
      color.g = 0.0;
      color.b = 1.0;
      color.a = 0.5;
      visualization_msgs::MarkerArray markers;

      /* Get the contact ponts and display them as markers */
      collision_detection::getCollisionMarkersFromContacts(markers, "panda_link0", c_res.contacts, color,
                                                           ros::Duration(),  // remain until deleted
                                                           0.01);            // radius
      publishMarkers(markers);
    }
  }
  // END_SUB_TUTORIAL
  else
  {
    ROS_INFO("Not colliding");

    // delete the old collision point markers
    visualization_msgs::MarkerArray empty_marker_array;
    publishMarkers(empty_marker_array);
  }


  ros::spin();
  delete g_planning_scene;
  delete g_marker_array_publisher;
  ros::shutdown();
  return 0;
}