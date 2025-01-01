//poses_detection_publisher.cpp

#include <server_utils/poses_detection_publisher.h>

#include <assignment_1/Detections.h>
#include <assignment_1/CheckID.h>
#include <assignment_1/FinalTagList.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


PosesDetectionPublisher::PosesDetectionPublisher(ros::NodeHandle node) 
    : node_(node), tfListener_(tfBuffer_) { // Initialize the TransformListener with tfBuffer_
    this->publisher_ = this->node_.advertise<assignment_1::Detections>("/poses_detection", 1000);
    this->tag_list_service_ = this->node_.advertiseService("get_final_tag_list", &PosesDetectionPublisher::getFinalTagListCallback, this);
    ROS_INFO("PosesDetectionPublisher initialized.");
}


bool PosesDetectionPublisher::checkID(ros::NodeHandle& node, int id) {
    ros::ServiceClient client = node.serviceClient<assignment_1::CheckID>("/check_id");
    assignment_1::CheckID srv;
    srv.request.id = id;

    if (client.call(srv)) {
        return srv.response.is_requested;
    } else {
        ROS_ERROR("Failed to call check_id service.");
        return false;
    }
}


geometry_msgs::Pose PosesDetectionPublisher::transformPose(const geometry_msgs::Pose& input_pose, const std::string& source_frame, const std::string& target_frame) {
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::Pose transformed_pose;
    try {
        geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(3.0));
        tf2::doTransform(input_pose, transformed_pose, transformStamped);
        ROS_INFO("Transformation successful: %s -> %s", source_frame.c_str(), target_frame.c_str());
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("Transform failed: %s", ex.what());
    }
    return transformed_pose;
}


void PosesDetectionPublisher::publish() {
    // Wait for detections from the "/tag_detections" topic
    apriltag_ros::AprilTagDetectionArray::ConstPtr detections =
        ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", node_);

    //ROS_INFO("Checking /tag_detections...");
    
    if (!detections) {
        //ROS_WARN("No detections received from /tag_detections.");
        ROS_WARN("\n");
        return;
    }
    //ROS_INFO("Processing %ld detections", detections->detections.size());
    assignment_1::Detections msg;

    for (const auto& detection : detections->detections) {
        // Access pose data directly from the detection
        geometry_msgs::PoseWithCovarianceStamped poseCovarianceStamped = detection.pose;
        geometry_msgs::PoseWithCovariance poseCovariance = poseCovarianceStamped.pose;
        
        assignment_1::Detection detectionMessage;
        detectionMessage.pose = transformPose(poseCovariance.pose, "xtion_rgb_optical_frame", "map"); // Direct transformation
        detectionMessage.id = detection.id.at(0);     // Set the ID
        msg.poses.push_back(detectionMessage);

        bool is_requested = checkID(node_, detectionMessage.id);
        tag_data_[detectionMessage.id] = {detectionMessage.pose, is_requested};

        //ROS_INFO("Raw Detection: X: %f, Y: %f, Z: %f",
        //         poseCovariance.pose.position.x,
        //         poseCovariance.pose.position.y,
        //         poseCovariance.pose.position.z);

        ROS_INFO("Detected ID: %d and Status: %s",
                 detectionMessage.id,
                 is_requested ? "Requested" : "Not Requested");

        ROS_INFO("Transformed Pose: X: %f, Y: %f \n\n",
                 detectionMessage.pose.position.x,
                 detectionMessage.pose.position.y);
    }

    // Publish the detections
    publisher_.publish(msg);
    //ROS_INFO("Published detections.");
}


// New functionality: Service to get the final tag list
bool PosesDetectionPublisher::getFinalTagListCallback(assignment_1::FinalTagList::Request &req,
                                                      assignment_1::FinalTagList::Response &res) {

    // Add responses
    for (std::map<int, TagInfo>::iterator it = tag_data_.begin(); it != tag_data_.end(); ++it) {
        int id = it->first;
        const TagInfo &info = it->second;

        res.ids.push_back(id);
        res.poses.push_back(info.pose);
        res.statuses.push_back(info.status);
    }

    ROS_INFO("Final tag list sent to client.");
    return true;
}