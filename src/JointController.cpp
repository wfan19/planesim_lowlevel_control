#include "../include/planesim_lowlevel_control/JointController.hpp"

JointController::JointController(ros::NodeHandle nh, string _namespace)
    : _namespace(_namespace)
{
    this->n = nh;

    map<string, int>::iterator iterator;
    int counter;
    for (iterator = jointsMap.begin(); iterator != jointsMap.end(); ++iterator){
        Joint joint(iterator->first, counter == 0 ? Type::velocity : Type::position);
        joints[counter] = joint;

        boost::function<void (const std_msgs::Float64&)> callback = 
            [&](const std_msgs::Float64 &target){
                joints[counter].target = target.data;
                ROS_INFO("Received command %d for joint %s", target.data, iterator->first);        
        };
        string topicName = _namespace + "/" + iterator->first; 
        ROS_INFO("Subscribing to topic %s", topicName.c_str());
        joints[counter].subscriber = n.subscribe<std_msgs::Float64>(topicName, 1000, callback);
    }

}

JointController::~JointController(){
}

void JointController::init(){

    ROS_INFO("Spinning node!");
    ros::spin();
}

void JointController::onJointState(const sensor_msgs::JointState::ConstPtr &jointStatePtr){

    int counter = 0;

    vector<double> position = jointStatePtr->position;
    vector<double> velocity = jointStatePtr->velocity;

    for (string name : jointStatePtr->name){

    }
}