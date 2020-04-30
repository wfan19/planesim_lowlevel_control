#include "../include/planesim_lowlevel_control/JointController.hpp"

JointController::JointController(ros::NodeHandle nh){
    this->n = nh;
    for (ros::Time time : lastUpdateTimes){
        time = ros::Time::now();
    }
    propellerLastUpdateTime = ros::Time::now();

    jointStateSub = n.subscribe("/planesim/joint_state", 1000, &JointController::onJointState, this);
}

JointController::~JointController(){
}

void JointController::init(){
    ROS_INFO("Spinning node!");
    ros::spinOnce;
}

void JointController::onJointState(const sensor_msgs::JointState::ConstPtr &jointStatePtr){

    int counter = 0;

    std::vector<double> position = jointStatePtr->position;
    std::vector<double> velocity = jointStatePtr->velocity;

    for (std::string name : jointStatePtr->name){
        int index = jointsMap.find(name)->second;
        float dt = ros::Time::now().toSec - lastUpdateTimes[index].toSec;

        CMDs[index] = PIDs[index].update(Targets[index], index == 0 ? velocity[counter] : position[counter], dt);

        counter++;
        lastUpdateTimes[index] = ros::Time::now();
    }
}