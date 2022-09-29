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
        double _endTimeExp1Vid1;
        double _resetWaitTime;
        double _startingRollAngleVid1;
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
    _configSrv.request.model_name = "CraneBot";                             // Model name
    _configSrv.request.urdf_param_name = "robot_description";               // URDF param name (unused)
    std::vector<std::string> name_vec(3);                                   // Joint names vector
    name_vec.push_back("cables_joint_x");
    name_vec.push_back("cables_joint_y");
    name_vec.push_back("cables_joint_z");
    _configSrv.request.joint_names = name_vec;

    // Initializations
    _gazeboTime = 0;
    _resetWaitTime = 5;                                                     // Time to wait while positioning the system to a nonrest configuration [sec]

//     _endTimeExp1Vid1 = 18.60;                                                   // [sec]
//     _startingRollAngleVid2Exp4 = 0.1876;                                        // [rad]

    _video1Flag = true;
    _video2Flag = true;
}

// Callback for saving the pose of the links
void IDENTIFICATION::linkStateCB(gazebo_msgs::LinkStates lstate) {
    _platform_link_pose[0] = lstate.pose[6].position.x;
    _platform_link_pose[1] = lstate.pose[6].position.y;
    _platform_link_pose[2] = lstate.pose[6].position.z;
    _platform_link_pose[3] = lstate.pose[6].orientation.x;
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


void IDENTIFICATION::readMeasures() {

        double startTime = _gazeboTime;
        double elapsedTime = 0;
        double currentTime = 0;
        std::ofstream simdatafile;
        std::vector<double> pos_vec(3);                                         // Joint position [rad] vector

        // Define Sampling rate
        ros::Rate r(50);

    // ###############################################################

    if (_video1Flag == true) {
        
        // Open file where the measurement from the simulation will be saved
        // simdatafile.open ("/home/giancarlo/ros_ws/src/dual-arm-aerial-teleop/Software/catkin_ws/src/licasa1_identification/bag/simDataVid1NatRespX.txt");
            
        // Initialization for time sampling
        startTime = _gazeboTime;
        elapsedTime = 0;
        currentTime = 0;

        // Define the desired starting joint position
        pos_vec.push_back(0.0);
        pos_vec.push_back(0.0864);
        pos_vec.push_back(0.0);
        _configSrv.request.joint_positions = pos_vec;

        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        std::cout << _gazeboTime << std::endl;
        startTime = _gazeboTime;


        // Move the robot to a non-rest initial configuration by calling the gazebo/setModelConfiguration service
        while (elapsedTime<=5) {
            currentTime = _gazeboTime;
            elapsedTime = currentTime - startTime;
            std::cout << elapsedTime << std::endl;
            _setModelConfigurationClient.call(_configSrv);
            r.sleep();
        }
        std::cout << "Server says [ " << _configSrv.response.status_message << " ] " << std::endl;

        // Reset time variables
        startTime = _gazeboTime;
        elapsedTime = 0;
        currentTime = 0;

        // Save the time instant and the pose of the platform link on a text file
        while (ros::ok() && elapsedTime<=20) {
            currentTime = _gazeboTime;
            elapsedTime = currentTime - startTime;
            std::cout << elapsedTime << std::endl;
            // simdatafile << elapsedTime << " ";
            // simdatafile << _platform_link_pose[0] << " ";
            // simdatafile << _platform_link_pose[1] << " ";
            // simdatafile << _platform_link_pose[2] << " ";
            // simdatafile << _platform_link_pose[3] << " ";
            // simdatafile << _platform_link_pose[4] << " ";
            // simdatafile << _platform_link_pose[5] << " ";
            // simdatafile << _platform_link_pose[6] << " ";
            // simdatafile << std::endl;

            r.sleep();
        }
        // simdatafile.close();

    }

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