#ifndef JOINTCONTROLLER_HPP
#define JOINTCONTROLLER_HPP

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

#include <thread>
#include <chrono>
#include <array>
#include <string>
#include <map>
#include <functional>

#include "PIDFF.hpp"

using namespace std;
class JointController {
public:
    JointController(ros::NodeHandle nh);
    ~JointController();

    void init();

    void onJointState(const sensor_msgs::JointStateConstPtr &jointState);

private:
    ros::NodeHandle n;
    
    map<string, int> jointsMap = {
        {"propeller_joint", 0},
        {"left_aileron_joint", 1},
        {"right_aileron_joint", 2},
    };

    static constexpr int jointCount = 3;

    array<ros::Time, jointCount> lastUpdateTimes;
    array<float, jointCount> targets;
    array<PIDFF, jointCount> PIDs;
    array<float, jointCount> CMDs;

    ros::Time propellerLastUpdateTime;
    float propellerTarget;
    PIDFF propellerPID;
    float propellerCMD;

    ros::Subscriber jointStateSub;
    array<ros::Subscriber, jointCount> targetSubs;
};

#endif