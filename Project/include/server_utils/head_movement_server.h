#pragma once

#include <ros/ros.h>
#include <ros/topic.h>
#include <actionlib/client/simple_action_client.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <assignment_1/HeadMovement.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> HeadControllClient;

class HeadMovementServer
{
public:
    // Constructor
    HeadMovementServer();
    // Service callback function
    bool headMovementCallback(assignment_1::HeadMovement::Request &req, 
                              assignment_1::HeadMovement::Response &res);

private:
    HeadControllClient headClient;                // Action client for moving the head
    control_msgs::FollowJointTrajectoryGoal goal; // Goal for head movement
    ros::NodeHandle node;                         // ROS NodeHandle
    ros::ServiceServer server;                    // ROS ServiceServer
};

