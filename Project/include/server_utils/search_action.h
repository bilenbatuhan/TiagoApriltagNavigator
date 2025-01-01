#pragma once

#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"

#include <actionlib/server/simple_action_server.h>
#include <assignment_1/SearchAction.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <cmath>

#include <geometry/polar_point.h>
#include <geometry/cartesian_point.h>
#include <geometry/position.h>

#include <server_utils/search_action.h>
#include <server_utils/poses_detection_publisher.h>

#include "nav_msgs/OccupancyGrid.h"

#include <cstdlib>
#include <string>
#include <algorithm>
#include <iostream>


class SearchAction {
private:
    ros::NodeHandle nh_;                                                // ROS node handle
    actionlib::SimpleActionServer<assignment_1::SearchAction> as_;   // action server
    std::string action_name_;                                           // action name  
    assignment_1::SearchFeedback feedback_;                          // feedback message                   
    assignment_1::SearchResult result_;                              // result message
    assignment_1::Detections current_detections_; // Store the latest detections
    PosesDetectionPublisher pose_publisher_;                           // Pose detection publisher
    ros::Subscriber detection_subscriber_; // Subscriber for detections

    
public:

    SearchAction(std::string name, ros::NodeHandle nh);
    void executeCB(const assignment_1::SearchGoalConstPtr &goal);
    void detectionCallback(const assignment_1::Detections::ConstPtr& msg); // Callback for detections

};
