#pragma once

#include "ros/ros.h"
#include "assignment_1/CheckID.h"
#include "assignment_1/SetRequestedIDs.h"
#include <vector>

class CheckIDServer {
public:
    explicit CheckIDServer(ros::NodeHandle& nh);

private:
    ros::NodeHandle nh_;
    ros::ServiceServer check_id_service_;
    ros::ServiceServer set_requested_ids_service_;
    std::vector<int> requested_ids_;

    bool checkIDCallback(assignment_1::CheckID::Request &req,
                         assignment_1::CheckID::Response &res);

    bool setRequestedIDsCallback(assignment_1::SetRequestedIDs::Request &req,
                                 assignment_1::SetRequestedIDs::Response &res);
};

//#endif // CHECK_ID_SERVER_H
