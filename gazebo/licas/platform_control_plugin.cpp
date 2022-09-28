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
		private: physics::JointPtr _cable_x_joint;
		private: physics::JointPtr _cable_y_joint;
		private: physics::JointPtr _shoulder_x_joint;
		private: physics::JointPtr _shoulder_y_joint;
		private: ros::Publisher _command_sx_pub;
		private: ros::Publisher _command_sy_pub;

		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
			printf("The plugin has been correctly loaded!\n");
			_node_handle = new ros::NodeHandle();
			_model = _parent;
			_cable_x_joint = this->model->GetJoint("revolute_joint_x");
			_cable_y_joint = this->model->GetJoint("revolute_joint_y");
			_shoulder_x_joint = this->model->GetJoint("shoulder_joint_x");
			_shoulder_y_joint = this->model->GetJoint("shoulder_joint_y");
			_command_sx_pub = _node_handle->advertise< std_msgs::Float64 >("/licasa1/licasa1_shoulder_x_effort_pos_controller/command", 0);
			_command_sy_pub = _node_handle->advertise< std_msgs::Float64 >("/licasa1/licasa1_shoulder_y_effort_pos_controller/command", 0);
			this->_updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&PlatformControlPlugin::OnUpdate, this));
		}

		// Called by the world update start event
		public: void OnUpdate() {
			std_msgs::Float64 command_sx;
			std_msgs::Float64 command_sy;

			command_sx.data = -(_passive_x_joint->Position(0));
			command_sy.data = -(_passive_y_joint->Position(0));

			_command_sx_pub.publish(command_sx);
			_command_sy_pub.publish(command_sy);
		}

	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(PlatformControlPlugin)
}