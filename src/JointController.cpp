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
    int index;
    for (iterator = jointsMap.begin(); iterator != jointsMap.end(); ++iterator){
        // Initialize joint list
        joints[index] = Joint(iterator->first, index == 0 ? Type::velocity : Type::position);

        // Remap joint name to joint topic name
        // Joint name is the one used in JointState
        // Joint topic name is the one that all the topics for a joint will be under
        string jointName = iterator->first;
        string jointTopicName = _namespace + "/" + jointName + "_controller";

        // Subscriber callback
        boost::function<void (const std_msgs::Float64&)> callback = 
            [&, index](const std_msgs::Float64 &target){
                joints[index].target = target.data;
                ROS_INFO("Received command %f for joint %s", target.data, joints[index].name.c_str());        
        };

        // Register subscribers and callbacks
        ROS_INFO("Subscribing to topic %s", (jointTopicName + "/target").c_str());
        joints[index].subscriber = n.subscribe<std_msgs::Float64>(jointTopicName + "/target", 1000, callback);

        // Advertise publishers
        string pubTopicName = jointTopicName + "/command";
        ROS_INFO("Advertising to topic %s", (jointTopicName + "/command").c_str());
        joints[index].publisher = n.advertise<std_msgs::Float64>(jointTopicName + "/command", 100);

        index++;
    }

    jointStateSub = n.subscribe(_namespace + "/joint_states", 1000, &JointController::onJointState, this);

    ROS_INFO("Spinning node!");
    ros::spin();
}

void JointController::onJointState(const sensor_msgs::JointState::ConstPtr &jointStatePtr){

    vector<double> position = jointStatePtr->position;
    vector<double> velocity = jointStatePtr->velocity;

    int counter = 0; // Tracks index in the position/verlocity vectors
    for (string name : jointStatePtr->name){
        int index = jointsMap.find(name)->second;

        Joint *joint = &joints[index];

        joint->current = position[counter];

        double currentTime = ros::Time::now().toSec();
        double dt = currentTime - joint->lastUpdateTime;
        double controlEffort = joint->controller.update(joint->target, joint->current, dt);

        ROS_INFO("Control effort for joint %s: %f", joint->name.c_str(), controlEffort);

        std_msgs::Float64 controlMessage;
        controlMessage.data = controlEffort;
        joint->publisher.publish(controlMessage);

        counter++;
    }
}