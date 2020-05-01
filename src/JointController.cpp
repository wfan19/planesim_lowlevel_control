#include "../include/planesim_lowlevel_control/JointController.hpp"

JointController::JointController(ros::NodeHandle nh, string _namespace)
    : _namespace(_namespace)
{
    this->n = nh;
}

JointController::~JointController(){
}

void JointController::init(){

    // We iterate through jointsMap so that joints[] will be in the order of jointsMap
    map<string, int>::iterator iterator;
    int counter;
    for (iterator = jointsMap.begin(); iterator != jointsMap.end(); ++iterator){
        // Initialize joint list
        joints[counter] = Joint(iterator->first, counter == 0 ? Type::velocity : Type::position);

        // Remap joint name to joint topic name
        // Joint name is the one used in JointState
        // Joint topic name is the one that all the topics for a joint will be under
        string jointName = iterator->first;
        string jointTopicName = _namespace + "/" + jointName.substr(0, jointName.length - 6) + "_controller";

        // Subscriber callback
        boost::function<void (const std_msgs::Float64&)> callback = 
            [&, counter](const std_msgs::Float64 &target){
                joints[counter].target = target.data;
                ROS_INFO("Received command %f for joint %s", target.data, joints[counter].name.c_str());        
        };

        // Register subscribers and callbacks
        ROS_INFO("Subscribing to topic %s", (jointTopicName + "/target").c_str());
        joints[counter].subscriber = n.subscribe<std_msgs::Float64>(jointTopicName + "/target", 1000, callback);

        // Advertise publishers
        string pubTopicName = jointTopicName + "/command";
        ROS_INFO("Advertising to topic %s", (jointTopicName + "/target").c_str());
        joints[counter].publisher = n.advertise<std_msgs::Float64>(jointTopicName + "/command", 100);

        counter++;
    }

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