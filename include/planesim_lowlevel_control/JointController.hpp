#ifndef JOINTCONTROLLER_HPP
#define JOINTCONTROLLER_HPP

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include <thread>
#include <chrono>
#include <array>
#include <string>
#include <map>

#include "../include/planesim_lowlevel_control/PIDFF.hpp"

class JointController {
public:
    JointController(ros::NodeHandle nh);
    ~JointController();

    void init();

    void onJointState(const sensor_msgs::JointStateConstPtr &jointState);

private:
    ros::NodeHandle n;
    
    const std::map<std::string, int> jointsMap = {
        {"propeller_joint", 0},
        {"left_aileron_joint", 1},
        {"right_aileron_joint", 2},
    };

    std::array<ros::Time, 3> lastUpdateTimes;
    std::array<float, 3> Targets;
    std::array<PIDFF, 3> PIDs;
    std::array<float, 3> CMDs;

    ros::Time propellerLastUpdateTime;
    float propellerTarget;
    PIDFF propellerPID;
    float propellerCMD;

    ros::Subscriber jointStateSub;
};

#endif