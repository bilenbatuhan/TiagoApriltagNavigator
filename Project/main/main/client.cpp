#include "ros/ros.h"
#include <assignment_1/SearchAction.h>
#include <assignment_1/HeadMovement.h>
#include <assignment_1/SetRequestedIDs.h>
#include <assignment_1/FinalTagList.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "tiago_iaslab_simulation/Objs.h"
#include <iostream>
#include <cstring>
#include <vector>

// Function to retrieve and display the final tag list
void displayFinalTagList(ros::NodeHandle& nh) {
    ros::ServiceClient final_tag_list_client = nh.serviceClient<assignment_1::FinalTagList>("get_final_tag_list");
    assignment_1::FinalTagList srv;

    if (final_tag_list_client.call(srv)) {
        ROS_INFO("FINAL DETCTED APRILTAG LIST:\n");

        ROS_INFO("Apriltags with REQUESTED IDs:");
        for (size_t i = 0; i < srv.response.ids.size(); ++i) {
            if (srv.response.statuses[i]) { // Requested IDs
                ROS_INFO("ID: %d Position - X: %f, Y: %f, Status: Requested",
                         srv.response.ids[i],
                         srv.response.poses[i].position.x,
                         srv.response.poses[i].position.y);
            }
        }

        ROS_INFO("OTHER Apriltags:");
        for (size_t i = 0; i < srv.response.ids.size(); ++i) {
            if (!srv.response.statuses[i]) { // Not Requested IDs
                ROS_INFO("ID: %d Position - X: %.3f, Y: %.3f, Status: Not Requested",
                         srv.response.ids[i],
                         srv.response.poses[i].position.x,
                         srv.response.poses[i].position.y);
            }
        }
    } else {
        ROS_ERROR("Failed to call service get_final_tag_list.");
    }
}

// Function to move the head
void moveHead(ros::NodeHandle nh) {
    ros::ServiceClient head_movement_client = nh.serviceClient<assignment_1::HeadMovement>("/head_movement");
    assignment_1::HeadMovement headMovementMessage;

    if (head_movement_client.call(headMovementMessage)) {
        ROS_INFO("Head movement command sent successfully.");
    } else {
        ROS_ERROR("Failed to call head movement service.");
    }
}

// Feedback callback
void feedbackCallback(const assignment_1::SearchFeedbackConstPtr& feedback) {

    ROS_INFO("ROBOT IS SEARCHING");
    // Print robot's current position
    ROS_INFO("At Position: X: %.3f, Y: %.3f, R: %.3f\n",
             feedback->position.X, feedback->position.Y, feedback->position.R);

    // Print detected AprilTags and their poses
    if (!feedback->detections.poses.empty()) {
        ROS_INFO("APRILTAG IS DETECTED:");
        for (const auto& detection : feedback->detections.poses) {
            ROS_INFO("ID: %d Position - X: %.3f, Y: %.3f\n",
                     detection.id,
                     detection.pose.position.x,
                     detection.pose.position.y);
        }
    }

    //ROS_INFO("-- END OF FEEDBACK --");
}

// Done and active callbacks
void doneCb(const actionlib::SimpleClientGoalState& state, const assignment_1::SearchResultConstPtr& result) {
    ROS_INFO("Action completed with state: %s", state.toString().c_str());
}
void activeCb() {
    ROS_INFO("Goal is now active.");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Node_A");
    ros::NodeHandle nh;

    // Get requested IDs from apriltag_ids_srv
    ros::ServiceClient ids_client = nh.serviceClient<tiago_iaslab_simulation::Objs>("/apriltag_ids_srv");
    tiago_iaslab_simulation::Objs ids_srv;
    ids_srv.request.ready = true;

    std::vector<int> requested_ids;

    if (ids_client.call(ids_srv)) {
        ROS_INFO("Retrieved requested IDs:");
        for (int id : ids_srv.response.ids) {
            requested_ids.push_back(id);
            ROS_INFO("ID: %d", id);
        }
    } else {
        ROS_ERROR("Failed to call /apriltag_ids_srv");
        return 1;
    }

    // Send requested IDs to check_id_server
    ros::ServiceClient set_ids_client = nh.serviceClient<assignment_1::SetRequestedIDs>("/set_requested_ids");
    assignment_1::SetRequestedIDs set_ids_srv;
    set_ids_srv.request.ids = requested_ids;

    if (set_ids_client.call(set_ids_srv)) {
        ROS_INFO("Successfully sent requested IDs to check_id_server");
    } else {
        ROS_ERROR("Failed to call /set_requested_ids service");
        return 1;
    }

    actionlib::SimpleActionClient<assignment_1::SearchAction> ac("Search", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started.");

    moveHead(nh);

    assignment_1::SearchGoal goal;
    std::string input;

    // Predefined waypoints
    struct Waypoint {
        double X, Y, R;  // Position: X, Y, Orientation: Radians
    };

    std::vector<Waypoint> waypoints = {
        {0, 0, 0},
        {0.1, -0.2, -0.5},
        {2, 0, 0},
        {5, 0, 0.5},
        {7, 0.2, 0.5},
        {8, 0, 1},
        {9, 0.7, 0},
        {10, 0.7, -1},
        {11, 1, 1},
        {12, 1, 0.2},
        {12.5, -0.5, -0.8},
        {12, -1.5, -2.5},
        {12, -2.7, -0.5},
        {11.5, -3.5, -2.5},
        {10.5, -4, -3},
        {9, -4, -3.2},
        {8, -3.5, -3.5},
        {8.5, -3, -5.5},
        {8.5, -2, -5},
        {8.5, -1.5, -3.5},
        {8, -1, -3},
        {7, 0, -3},
        {0, 0, 0}
    };

    std::cout << "Would you like to use predefined waypoints? (y/n): ";
    std::cin >> input;

    if (input == "y" || input == "Y") {
        ROS_INFO("Using predefined waypoints...");
        for (size_t i = 0; i < waypoints.size(); ++i) {
            ROS_INFO("Sending waypoint %lu: X = %.3f, Y = %.3f, R = %.3f", 
                     i + 1, waypoints[i].X, waypoints[i].Y, waypoints[i].R);

            // Set goal
            goal.goal.X = waypoints[i].X;
            goal.goal.Y = waypoints[i].Y;
            goal.goal.R = waypoints[i].R;

            ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCallback);
            ac.waitForResult();

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Waypoint %lu reached successfully!", i + 1);
            } else {
                ROS_ERROR("Failed to reach waypoint %lu!", i + 1);
                break;  // Stop if a waypoint fails
            }
        }
    } else {
        ROS_INFO("Entering manual goal input mode...");
        // Manual input mode
        while (ros::ok()) {
            try {
                std::cout << "Insert x coordinate (q to quit): ";
                std::cin >> input;
                if (input == "q") break;
                goal.goal.X = std::stod(input);

                std::cout << "Insert y coordinate (q to quit): ";
                std::cin >> input;
                if (input == "q") break;
                goal.goal.Y = std::stod(input);

                std::cout << "Insert orientation (radians, q to quit): ";
                std::cin >> input;
                if (input == "q") break;
                goal.goal.R = std::stod(input);

                ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCallback);
                ROS_INFO("Detection goal sent.");

                ac.waitForResult();

                if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("Detection completed successfully.");
                } else {
                    ROS_ERROR("Detection action failed.");
                }
            } catch (const std::invalid_argument& e) {
                ROS_ERROR("Invalid input. Please enter numeric values.");
            }
        }
    }

    ROS_INFO("SEARCHING IS ENDED...\n");

    // Call the function to display the final tag list
    displayFinalTagList(nh);

    return 0;
}