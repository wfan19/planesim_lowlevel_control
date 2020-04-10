#ifndef FOILCONTROLLER_HPP
#define FOILCONTROLLER_HPP

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

#include "PIDFF.hpp"

class FoilController{

public:
    FoilController(ros::NodeHandle nodeHandle);
    ~FoilController();
    
    void controlSurfaces();

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &jointStatePtr);
    void onLeftAileronSP(const std_msgs::Float64::ConstPtr &aileronSPPointer);

private:
    ros::NodeHandle nodeHandle;

    PIDFF leftAileronPIDFF;

    void upateControllers();

    ros::Time lastUpdateTime;
    double lastLeftAileronSP;
    double lastLeftAileronState;
};

#endif