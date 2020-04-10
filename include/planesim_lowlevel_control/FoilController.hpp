#ifndef FOILCONTROLLER_HPP
#define FOILCONTROLLER_HPP

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

#include "PIDFF.hpp"

class FoilController{

public:
    FoilController();
    ~FoilController();
    
    void init(int argc, char **argv);

    PIDFF left_aileron_pidff;

    static void jointStateCallback(const sensor_msgs::JointState::ConstPtr &jointStatePtr);

    static ros::Publisher left_aileron_effort_pub;
    static ros::Subscriber joint_state_sub;
    // static ros::Publisher right_aileron_effort_pub;

};

#endif