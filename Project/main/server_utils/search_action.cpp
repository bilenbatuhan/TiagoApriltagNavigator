#include <server_utils/search_action.h>
#include <iostream>
#include <mutex>

//global variables for callback usage only *************************************

//keep track of the feedback loop
bool send_feedback = true;

//keep track of robot position
Position currentPosition = {};

// Mutex for thread-safe access to current detections
std::mutex detection_mutex;
assignment_1::Detections current_detections_;

//callback functions **********************************************************

void positionCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& m){
	double x = m->feedback.base_position.pose.position.x;
	double y = m->feedback.base_position.pose.position.y;
	
	double sin = m->feedback.base_position.pose.orientation.z;
	double cos = m->feedback.base_position.pose.orientation.w;
	
	double angle = Position::sinCosToRad(sin, cos);
	
	currentPosition.setPosition(x, y, angle);
}

void resultCallback(const move_base_msgs::MoveBaseActionResult &msg) {
	send_feedback = false;
}

SearchAction::SearchAction(std::string name, ros::NodeHandle nh) : 
    as_(nh, name, boost::bind(&SearchAction::executeCB, this, _1),false), 
    action_name_(name),
    nh_(nh), 
    pose_publisher_(nh)  // Initialize pose detection publisher
{
    // Subscribe to detection and robot feedback topics
    detection_subscriber_ = nh_.subscribe("/poses_detection", 1000, &SearchAction::detectionCallback, this);
    as_.start();
}

// Callback to handle detections from poses_detection_publisher
void SearchAction::detectionCallback(const assignment_1::Detections::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(detection_mutex);
    current_detections_ = *msg; // Update the latest detections
}

void SearchAction::executeCB(const assignment_1::SearchGoalConstPtr &goal) 
{
    ros::Subscriber result_subscriber = nh_.subscribe("/move_base/result", 1, resultCallback);
    ros::Subscriber feedback_subscriber = nh_.subscribe("/move_base/feedback", 10, positionCallback);

    ros::Rate r(5);
    ros::Publisher goal_publisher = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);
    ros::Duration(1.0).sleep();

    move_base_msgs::MoveBaseActionGoal goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.goal.target_pose.header.frame_id = "map";

    tf2::Quaternion quat_rotation;
	quat_rotation.setRPY(0, 0, goal->goal.R);
	goal_msg.goal.target_pose.pose.orientation.x = quat_rotation.x();
	goal_msg.goal.target_pose.pose.orientation.y = quat_rotation.y();
	goal_msg.goal.target_pose.pose.orientation.z = quat_rotation.z();
	goal_msg.goal.target_pose.pose.orientation.w = quat_rotation.w();
    
    goal_msg.goal.target_pose.pose.position.z = 0.0;
    goal_msg.goal.target_pose.pose.position.x = goal->goal.X;
    goal_msg.goal.target_pose.pose.position.y = goal->goal.Y;

    goal_publisher.publish(goal_msg);
    ROS_INFO("Goal sent to robot.");

    send_feedback = true;
    bool success = true;

    while (send_feedback) {
        feedback_.position.X = currentPosition.getPoint().getX();
        feedback_.position.Y = currentPosition.getPoint().getY();
        feedback_.position.R = currentPosition.getOrientation();

        // Safely update detections
        {
            std::lock_guard<std::mutex> lock(detection_mutex);
            feedback_.detections = current_detections_;
        }

        if (as_.isPreemptRequested() || !ros::ok()) {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_.setPreempted();
            success = false;
            break;
        }

        as_.publishFeedback(feedback_);
        pose_publisher_.publish();
        r.sleep();
    }

    if (success) {
        ROS_INFO("Result arrived from robot.");
        as_.setSucceeded(result_);
    } else {
        ROS_INFO("Action failed.");
    }
}

