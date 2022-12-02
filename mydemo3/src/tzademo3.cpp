#include <ros/ros.h>
#include <string.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>


void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";

    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.4;
    collision_objects[0].primitives[0].dimensions[1] = 0.2;
    collision_objects[0].primitives[0].dimensions[2] = 0.05;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0;
    collision_objects[0].primitive_poses[0].position.y = 0.2;
    collision_objects[0].primitive_poses[0].position.z = 0.35;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    planning_scene_interface.applyCollisionObjects(collision_objects);

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "tzademo2");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string Planning_group = "gluon";
    moveit::planning_interface::MoveGroupInterface move_group_interface(Planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    addCollisionObjects(planning_scene_interface);
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

    move_group_interface.setPoseReferenceFrame("dummy");
    move_group_interface.allowReplanning(true);
    move_group_interface.setGoalPositionTolerance(0.001);
    move_group_interface.setGoalOrientationTolerance(0.01);
    move_group_interface.setMaxAccelerationScalingFactor(1);
    move_group_interface.setMaxVelocityScalingFactor(1);

    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 0.5;
    target_pose.orientation.x = -0.5;
    target_pose.orientation.y = 0.5;
    target_pose.orientation.z = 0.5;

    target_pose.position.x = -0.05;
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.2;
    move_group_interface.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("PLANNING IS START");
    
    move_group_interface.execute(my_plan.trajectory_);
    ROS_INFO("TRAJECTORY IS START");

    ros::shutdown();
    return 0;

}