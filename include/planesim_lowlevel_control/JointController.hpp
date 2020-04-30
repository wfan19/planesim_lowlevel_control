#ifndef JOINTCONTROLLER_HPP
#define JOINTCONTROLLER_HPP

#include "ros/ros.h"

#include <thread>

class JointController {
public:
    JointController(ros::NodeHandle nh);
    ~JointController();

    void start(unsigned int updateIntervalMS);
    void stop();

private:
    ros::NodeHandle n;

    std::atomic<bool> running{false};
    std::thread mainThread;

};

#endif