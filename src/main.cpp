#include "ros/ros.h"
#include "../include/planesim_lowlevel_control/JointController.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "planesim_lowlevel_control");
    ros::NodeHandle n;

    JointController jointController(n);

    return 0;
}