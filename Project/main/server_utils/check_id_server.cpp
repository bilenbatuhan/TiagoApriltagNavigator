#include <server_utils/check_id_server.h>
#include <algorithm>

CheckIDServer::CheckIDServer(ros::NodeHandle& nh) : nh_(nh) {
    // Advertise services
    check_id_service_ = nh_.advertiseService("check_id", &CheckIDServer::checkIDCallback, this);
    set_requested_ids_service_ = nh_.advertiseService("set_requested_ids", &CheckIDServer::setRequestedIDsCallback, this);

    ROS_INFO("Check ID Server initialized.");
}

// Callback to check ID
bool CheckIDServer::checkIDCallback(assignment_1::CheckID::Request &req,
                                    assignment_1::CheckID::Response &res) {
    ROS_INFO("Checking if id %d is requested...", req.id);

    if (requested_ids_.empty()) {
        ROS_WARN("Requested IDs list is empty.");
        res.is_requested = false;
        return true;
    }

    res.is_requested = std::find(requested_ids_.begin(), requested_ids_.end(), req.id) != requested_ids_.end();
    ROS_INFO("ID %d is %sREQUESTED.", req.id, res.is_requested ? "" : "NOT ");
    return true;
}

// Callback to set requested IDs
bool CheckIDServer::setRequestedIDsCallback(assignment_1::SetRequestedIDs::Request &req,
                                            assignment_1::SetRequestedIDs::Response &res) {
    requested_ids_ = req.ids;
    ROS_INFO("Requested IDs:");
    for (int id : requested_ids_) {
        ROS_INFO("ID: %d", id);
    }
    res.success = true;
    return true;
}
