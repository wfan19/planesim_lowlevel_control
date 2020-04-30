#include "../include/planesim_lowlevel_control/JointController.hpp"

JointController::JointController(ros::NodeHandle nh){
    this->n = nh;
}

JointController::~JointController(){
}

void JointController::start(unsigned int updateIntervalMS){
    
}