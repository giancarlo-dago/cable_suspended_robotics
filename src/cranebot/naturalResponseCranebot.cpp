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
        double _platform_link_pose[7];
        double _gazeboTime;
        double _resetWaitTime;
    public:
        IDENTIFICATION();
        void run();
        void readMeasures();
        void linkStateCB(gazebo_msgs::LinkStates);          // Link state callback
        void gazeboClockCB(rosgraph_msgs::Clock);           // Gazebo clock callback
};


// Constructor
IDENTIFICATION::IDENTIFICATION() {
    _pauseGazeboClient = _nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");                                               // Pause Gazebo Client
    _unpauseGazeboClient = _nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");                                           // Unpause Gazebo Client
    _setModelConfigurationClient = _nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");        // Set Model Configuration Client
    _linkStateSub = _nh.subscribe("/gazebo/link_states",0,&IDENTIFICATION::linkStateCB,this);                                       // Link State subscriber
    _gazeboClockSub = _nh.subscribe("/clock",0,&IDENTIFICATION::gazeboClockCB,this);                                                // Gazebo Clock Subscriber

    // Definition of the service request message
    _configSrv.request.model_name = "CraneBot";                             // Model name
    _configSrv.request.urdf_param_name = "robot_description";               // URDF param name (unused)

    // Definition of the joint names and initial position vectors
    std::vector<std::string> joints_name_vec;                               // Joint names [rad] vector
    std::vector<double> joints_pos_vec;                                     // Joint position [rad] vector
    joints_name_vec.insert(joints_name_vec.end(), {"cables_joint_z","cables_joint_x","cables_joint_y"});
    joints_pos_vec.insert(joints_pos_vec.end(), {0, 0, 0.2});      // [rad]

    // Configuration of the service
    _configSrv.request.joint_names = joints_name_vec;                       // Configuring the server with joints name list
    _configSrv.request.joint_positions = joints_pos_vec;

    // Initializations
    _gazeboTime = 0;                                                        // Initializing Gazebo time variable
    _resetWaitTime = 5;                                                     // Time to wait while positioning the system to a nonrest configuration [sec]
}


// Callback for saving the pose of the links
void IDENTIFICATION::linkStateCB(gazebo_msgs::LinkStates lstate) {
    _platform_link_pose[0] = lstate.pose[6].position.x;                     // Position
    _platform_link_pose[1] = lstate.pose[6].position.y;
    _platform_link_pose[2] = lstate.pose[6].position.z;
    _platform_link_pose[3] = lstate.pose[6].orientation.x;                  // Orientation in quaternions
    _platform_link_pose[4] = lstate.pose[6].orientation.y;
    _platform_link_pose[5] = lstate.pose[6].orientation.z;
    _platform_link_pose[6] = lstate.pose[6].orientation.w;
}


// Callback for saving time from gazebo. Conversion in second
void IDENTIFICATION::gazeboClockCB(rosgraph_msgs::Clock clockMsg) {
    double sec = clockMsg.clock.sec;
    double nsec = clockMsg.clock.nsec;
    double num = 1000000000;
    _gazeboTime = sec+nsec/num;
}


// Modify the initial state of the system
void IDENTIFICATION::readMeasures() {

    // Define Sampling rate
    ros::Rate r(50);

    // Initialization for data saving (to be inserted)
    std::ofstream simdatafile;

    // Initialization for time sampling
    double startTime = _gazeboTime;
    double elapsedTime = 0; 
    double currentTime = 0;
    double oldElapsed = 0;

    // Move the robot to a non-rest initial configuration by calling the gazebo/setModelConfiguration service and waiting _resetWaitTime seconds
    for (int i=0; i<2; i++) {                                       // In the first loop the value of elapsed time is wrong (not sure why) so two loops are needed
        while (elapsedTime<=oldElapsed+_resetWaitTime) {
            currentTime = _gazeboTime; 
            elapsedTime = currentTime - startTime;
            _setModelConfigurationClient.call(_configSrv);          // Set model initial state
            r.sleep();
        }
        oldElapsed = elapsedTime;    
    }
    std::cout << "Server says [ " << _configSrv.response.status_message << " ] " << std::endl;

    // Shutdown ros
    ros::shutdown();
}


// Run loop
void IDENTIFICATION::run() {
    boost::thread readMeasures_t(&IDENTIFICATION::readMeasures,this);
    ros::spin();
}


// Main loop
int main (int argc, char** argv) {
    ros::init(argc, argv, "identification");
    IDENTIFICATION identification;
    ros::Duration(0.5).sleep(); // sleep for half a second
    identification.run();
    return 0;
}