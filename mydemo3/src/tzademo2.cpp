#include <ros/ros.h>
#include <string.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tzademo2");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string Planning_group = "gluon";
    moveit::planning_interface::MoveGroupInterface move_group_interface(Planning_group);

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

    target_pose.position.x = -0.0;
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