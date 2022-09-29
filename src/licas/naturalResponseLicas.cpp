#include "ros/ros.h"
#include "boost/thread.hpp"
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include "rosgraph_msgs/Clock.h"
#include <string>
#include <iostream>
#include <fstream>
#include <stack>
#include <chrono>


class IDENTIFICATION {
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _linkStateSub;
        ros::Subscriber _gazeboClockSub;
        ros::ServiceClient _pauseGazeboClient;
        ros::ServiceClient _unpauseGazeboClient;
        ros::ServiceClient _setModelConfigurationClient;
        std_srvs::Empty _pauseSrv;
        std_srvs::Empty _unpauseSrv;
        gazebo_msgs::SetModelConfiguration _configSrv;
        double _shoulder_link_pose[7];
        double _gazeboTime;
        double _resetWaitTime;
        double _startingAngle1;
        double _startingAngle2;
        double _startingAngle3;
        double _startingAngle4_1;
        double _startingAngle4_2;
        bool _video1Flag;
        bool _video2Flag;        
    public:
        IDENTIFICATION();
        void run();
        void readMeasures();
        void linkStateCB(gazebo_msgs::LinkStates);
        void gazeboClockCB(rosgraph_msgs::Clock);
};


IDENTIFICATION::IDENTIFICATION() {
    _pauseGazeboClient = _nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    _unpauseGazeboClient = _nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    _setModelConfigurationClient = _nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    _linkStateSub = _nh.subscribe("/gazebo/link_states",0,&IDENTIFICATION::linkStateCB,this);
    _gazeboClockSub = _nh.subscribe("/clock",0,&IDENTIFICATION::gazeboClockCB,this);

    // Definition of the service request message
    _configSrv.request.model_name = "LiCAS_A1";                             // Model name
    _configSrv.request.urdf_param_name = "robot_description";               // URDF param name (unused)
    std::vector<std::string> name_vec(6);                                   // Joint names vector
    name_vec.push_back("revolute_joint_x");
    name_vec.push_back("revolute_joint_y");
    name_vec.push_back("revolute_joint_z");
    name_vec.push_back("shoulder_joint_x");
    name_vec.push_back("shoulder_joint_y");
    name_vec.push_back("shoulder_joint_z");
    _configSrv.request.joint_names = name_vec;

    // Initializations
    _gazeboTime = 0;
    _resetWaitTime = 5;                                                     // Time to wait while positioning the system to a nonrest configuration [sec]

    _startingAngle1 = 0.1983;                                               // [rad]
    _startingAngle2 = 0.2620;                                               // [rad]
    _startingAngle3 = 0.9741;                                               // [rad]
    _startingAngle4_1 = 0.1881;                                            // [rad]
    _startingAngle4_2 = 0.1678;                                             // [rad]

}

// Callback for saving the pose of the links
void IDENTIFICATION::linkStateCB(gazebo_msgs::LinkStates lstate) {
    _shoulder_link_pose[0] = lstate.pose[6].position.x;
    _shoulder_link_pose[1] = lstate.pose[6].position.y;
    _shoulder_link_pose[2] = lstate.pose[6].position.z;
    _shoulder_link_pose[3] = lstate.pose[6].orientation.x;
    _shoulder_link_pose[4] = lstate.pose[6].orientation.y;
    _shoulder_link_pose[5] = lstate.pose[6].orientation.z;
    _shoulder_link_pose[6] = lstate.pose[6].orientation.w;
}


// Callback for saving time from gazebo. Conversion in second
void IDENTIFICATION::gazeboClockCB(rosgraph_msgs::Clock clockMsg) {
    double sec = clockMsg.clock.sec;
    double nsec = clockMsg.clock.nsec;
    double num = 1000000000;
    _gazeboTime = sec+nsec/num;
}


void IDENTIFICATION::readMeasures() {

    double startTime = _gazeboTime;
    double elapsedTime = 0; 
    double currentTime = 0;
    std::ofstream simdatafile;
    std::vector<double> pos_vec(6);                                         // Joint position [rad] vector

    // Define Sampling rate
    ros::Rate r(50);

    // --------------------- FIRST EXPERIMENT ---------------------
        
    // Initialization for time sampling
    startTime = _gazeboTime;
    elapsedTime = 0; 
    currentTime = 0;

    // Define the desired starting joint position
    // pos_vec.push_back(_startingAngle1);
    // pos_vec.push_back(0.0);
    // pos_vec.push_back(0.0);
    // pos_vec.push_back(0.0);

    // pos_vec.push_back(0.0);
    // pos_vec.push_back(_startingAngle2);
    // pos_vec.push_back(0.0);
    // pos_vec.push_back(0.0);

    // pos_vec.push_back(0.0);
    // pos_vec.push_back(0.0);
    // pos_vec.push_back(_startingAngle3);
    // pos_vec.push_back(0.0);

    pos_vec.push_back(_startingAngle4_2);
    pos_vec.push_back(_startingAngle4_1);
    pos_vec.push_back(0.0);
    pos_vec.push_back(-_startingAngle4_2);
    pos_vec.push_back(-_startingAngle4_1);
    pos_vec.push_back(0.0);

    _configSrv.request.joint_positions = pos_vec;

    // Move the robot to a non-rest initial configuration by calling the gazebo/setModelConfiguration service
    while (elapsedTime<=_resetWaitTime) {
        currentTime = _gazeboTime; 
        elapsedTime = currentTime - startTime;
        _setModelConfigurationClient.call(_configSrv);
        std::cout << elapsedTime<< std::endl;
        r.sleep();
    }
    
    double oldElapsed = elapsedTime;

    while (elapsedTime<=oldElapsed+_resetWaitTime) {
        currentTime = _gazeboTime;
        elapsedTime = currentTime - startTime;
        _setModelConfigurationClient.call(_configSrv);
        std::cout << elapsedTime<< std::endl;
        r.sleep();
    }
    std::cout << "Server says [ " << _configSrv.response.status_message << " ] " << std::endl;

    ros::shutdown();


}


void IDENTIFICATION::run() {
    boost::thread readMeasures_t(&IDENTIFICATION::readMeasures,this);
    ros::spin();
}


int main (int argc, char** argv) {
    ros::init(argc, argv, "identification");
    IDENTIFICATION identification;
    ros::Duration(0.5).sleep(); // sleep for half a second
    identification.run();
    return 0;
}