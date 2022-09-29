#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

# define M_PI 3.14159265358979323846  /* pi */

using namespace std;
	
namespace gazebo {
	class PlatformControlPlugin : public ModelPlugin {
		private: ros::NodeHandle* _node_handle;
		private: physics::ModelPtr _model;
		private: event::ConnectionPtr _updateConnection;
		private: physics::JointPtr _cable_x_joint;				// Pointer to cable joint with rotation axis X
		private: physics::JointPtr _cable_y_joint;				// Pointer to cable joint with rotation axis Y
		private: ros::Publisher _command_sx_pub;				// Command publisher for lower platform joint X
		private: ros::Publisher _command_sy_pub;				// Command publisher for lower platform joint Y

		// Loaded at Gazebo startup
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
			printf("The plugin has been correctly loaded!\n");
			_node_handle = new ros::NodeHandle();
			_model = _parent;
			_cable_x_joint = this->_model->GetJoint("revolute_joint_x");							// Joint handle definition
			_cable_y_joint = this->_model->GetJoint("revolute_joint_y");							//
			_command_sx_pub = _node_handle->advertise< std_msgs::Float64 >("/licasa1/licasa1_shoulder_x_effort_pos_controller/command", 0);		// Publisher adversisers	
			_command_sy_pub = _node_handle->advertise< std_msgs::Float64 >("/licasa1/licasa1_shoulder_y_effort_pos_controller/command", 0);
			this->_updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&PlatformControlPlugin::OnUpdate, this));				// Plugin updater
		}

		// Called by the world update start event
		public: void OnUpdate() {
			std_msgs::Float64 command_sx;									// Define data structure for sending commands			
			std_msgs::Float64 command_sy;									//
			command_sx.data = -(_cable_x_joint->Position(0));				// Cable joint x position = -(lower platform joint x posion)
			command_sy.data = -(_cable_y_joint->Position(0)); 			// Cable joint y position = -(lower platform joint y posion)
			_command_sx_pub.publish(command_sx);							// The command are published
			_command_sy_pub.publish(command_sy);							//
		}

	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(PlatformControlPlugin)
}