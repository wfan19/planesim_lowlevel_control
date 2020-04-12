#include "../include/planesim_lowlevel_control/FoilController.hpp"


FoilController::FoilController(ros::NodeHandle nh)
    : nodeHandle(nh)
    , leftAileronPIDFF(3, 0, 0, 0, 0, 0, -3, 3){

    lastUpdateTime = ros::Time::now();

    leftAileronEffortPub = nodeHandle.advertise<std_msgs::Float64>("/planesim/left_aileron_effort_controller/command", 1000);
}

FoilController::~FoilController(){
}


void FoilController::controlSurfaces(){
    double dt = ros::Time::now().toSec() - lastUpdateTime.toSec();
    double error = lastLeftAileronSP - lastLeftAileronState;

    double leftAileronControlEffort = leftAileronPIDFF.update(lastLeftAileronSP, error, dt);
    std_msgs::Float64 leftAileronMsg;
    leftAileronMsg.data = leftAileronControlEffort;

    ROS_INFO("Publishing control effort %f", leftAileronControlEffort);
    leftAileronEffortPub.publish(leftAileronMsg);
}

void FoilController::jointStateCallback(const sensor_msgs::JointState::ConstPtr &jointStatePtr){
    double position0 = jointStatePtr->position[0];
    lastLeftAileronState = position0;
    // ROS_INFO("Joint state positions: %f", position0);
    controlSurfaces();
}

void FoilController::onLeftAileronSP(const std_msgs::Float64::ConstPtr &aileronSPPointer){
    lastLeftAileronSP = aileronSPPointer->data;
    controlSurfaces();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rcPub");

    ros::NodeHandle n;

    FoilController mFoilController(n);

    ros::Subscriber joint_state_sub = n.subscribe("/planesim/joint_states", 1000, &FoilController::jointStateCallback, &mFoilController);
    ros::Subscriber left_aileron_sp_sub = n.subscribe("planesim/setpoints/left_aileron_sp", 1000, &FoilController::onLeftAileronSP, &mFoilController);
    
    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}