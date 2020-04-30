#include "../include/planesim_lowlevel_control/JointController.hpp"

JointController::JointController(ros::NodeHandle nh){
    this->n = nh;
    for (ros::Time time : lastUpdateTimes){
        time = ros::Time::now();
    }
    propellerLastUpdateTime = ros::Time::now();

    jointStateSub = n.subscribe("/planesim/joint_state", 1000, &JointController::onJointState, this);

    map<string, int>::iterator iterator;
    int index = 0;
    for (iterator = jointsMap.begin(); iterator != jointsMap.end(); iterator++){
        
        boost::function<void (const std_msgs::Float64&)> callback = 
            [&](const std_msgs::Float64 &target){
                targets[index] = target.data;
                ROS_INFO("Received command %d for joint %s", target.data, iterator->second);        
        };
        
        targetSubs[index] = n.subscribe<std_msgs::Float64>("/planesim/" + iterator->first, 1000, callback);

    }
}

JointController::~JointController(){
}

void JointController::init(){
    ROS_INFO("Spinning node!");
    ros::spinOnce;
}

void JointController::onJointState(const sensor_msgs::JointState::ConstPtr &jointStatePtr){

    int counter = 0;

    vector<double> position = jointStatePtr->position;
    vector<double> velocity = jointStatePtr->velocity;

    for (string name : jointStatePtr->name){
        int index = jointsMap.find(name)->second;
        float dt = ros::Time::now().toSec - lastUpdateTimes[index].toSec;

        CMDs[index] = PIDs[index].update(targets[index], index == 0 ? velocity[counter] : position[counter], dt);

        counter++;
        lastUpdateTimes[index] = ros::Time::now();
    }
}