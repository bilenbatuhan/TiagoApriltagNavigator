#include "ros/ros.h"
#include <server_utils/search_action.h>
#include <server_utils/head_movement_server.h>
#include <server_utils/check_id_server.h>


int main(int argc, char **argv) {
    
    // Initialize ROS node
    //ros::init(argc, argv, "goal_receiver_node");
	ros::init(argc, argv, "Node_B");
	ros::NodeHandle nh;

	HeadMovementServer head_movement_server;
	SearchAction Search("Search", nh);
	CheckIDServer check_id_server(nh);
    
    	
	ros::spin();
	
    return 0;
}
