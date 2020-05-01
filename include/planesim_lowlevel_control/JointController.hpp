#ifndef JOINTCONTROLLER_HPP
#define JOINTCONTROLLER_HPP

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

#include <thread>
#include <chrono>
#include <array>
#include <string>
#include <map>
#include <functional>

#include "PIDFF.hpp"

using namespace std;
class JointController {
public:
    enum Type{
        position,
        velocity,
    };
    
    struct Joint{
        string name;
        Type type;

        ros::Subscriber subscriber;
        ros::Publisher publisher;

        PIDFF controller;
        double lastUpdateTime;
        double target = 0;
        double current = 0;

        Joint(string _name, Type _type){
            name = _name;
            type = _type;

            controller = PIDFF();
            lastUpdateTime = ros::Time::now().toSec();
        }
    };

    JointController(ros::NodeHandle nh, string _namespace);
    ~JointController();

    void init();

    void onJointState(const sensor_msgs::JointStateConstPtr &jointState);

private:
    ros::NodeHandle n;
    string _namespace;
    
    map<string, int> jointsMap = {
        {"propeller_joint", 0},
        {"left_wing_joint", 1},
        {"right_wing_joint", 2},
    };

    static constexpr int jointCount = 3;
    Joint joints[jointCount];

    ros::Subscriber jointStateSub;
};

#endif