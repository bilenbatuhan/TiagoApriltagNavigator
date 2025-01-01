#include <server_utils/head_movement_server.h>

HeadMovementServer::HeadMovementServer() : 
      headClient("/head_controller/follow_joint_trajectory", true)
{
    this->node = node;
    this->server = this->node.advertiseService(
        "/head_movement", 
        &HeadMovementServer::headMovementCallback, 
        this
    );
    this->goal.trajectory.joint_names.push_back("head_1_joint");
    this->goal.trajectory.joint_names.push_back("head_2_joint");
    this->goal.trajectory.points.resize(1); 
}

bool HeadMovementServer::headMovementCallback(assignment_1::HeadMovement::Request &req, assignment_1::HeadMovement::Response &res)
{
    // Resize to put the value of the joints
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    goal.trajectory.points.at(0).positions.resize(goal.trajectory.joint_names.size());
    
    ROS_INFO("\nHeadMovement started\n");
    
    goal.trajectory.points.at(0).positions.at(0) = 0.0;
    goal.trajectory.points.at(0).positions.at(1) = -2; 
    goal.trajectory.points.at(0).time_from_start = ros::Duration(2.0);

    headClient.sendGoal(goal);
    ros::Duration(3).sleep();
    res.state = true;

    return true;
}