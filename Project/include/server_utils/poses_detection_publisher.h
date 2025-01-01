#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <assignment_1/Detections.h>
#include <assignment_1/FinalTagList.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <map>


class PosesDetectionPublisher {
public:
    explicit PosesDetectionPublisher(ros::NodeHandle node);
    void publish();

private:
    ros::NodeHandle node_;
    ros::Publisher publisher_;
    ros::ServiceServer tag_list_service_; // Service for final tag list
    tf2_ros::Buffer tfBuffer_;            // Buffer for TF transformations
    tf2_ros::TransformListener tfListener_; // TF listener

    struct TagInfo {
        geometry_msgs::Pose pose;
        bool status; // True if requested, false otherwise
    };
    std::map<int, TagInfo> tag_data_; // Stores tag ID, pose, and status

    geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose, 
                                      const std::string& source_frame, 
                                      const std::string& target_frame);
    bool checkID(ros::NodeHandle& node, int id); // Utility function for ID check
    bool getFinalTagListCallback(assignment_1::FinalTagList::Request &req,
                                 assignment_1::FinalTagList::Response &res);
};