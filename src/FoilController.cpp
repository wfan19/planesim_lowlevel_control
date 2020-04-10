#include "../include/planesim_lowlevel_control/FoilController.hpp"

ros::Publisher FoilController::left_aileron_effort_pub;

ros::Subscriber FoilController::joint_state_sub;

FoilController::FoilController():
    left_aileron_pidff(10,0,0,0,0,0,-100,100){

}

FoilController::~FoilController(){
}

void FoilController::init(int argc, char **argv){
    ros::init(argc, argv, "rcPub");

    ros::NodeHandle n;

    left_aileron_effort_pub = n.advertise<std_msgs::Float64>("/planesim/left_aileron_effort_controller/command", 1000);

    joint_state_sub = n.subscribe("/planesim/joint_states", 1000, jointStateCallback);

    ROS_INFO("Spinning node");
    ros::spin();
}

void FoilController::jointStateCallback(const sensor_msgs::JointState::ConstPtr &jointStatePtr){
    double position0 = jointStatePtr->position[0];
    ROS_INFO("Joint state positions: %f", position0);
}

int main(int argc, char **argv){
    FoilController mFoilController;
    mFoilController.init(argc, argv);
    return 0;
}