#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

# define M_PI 3.14159265358979323846  /* pi */

using namespace std;
	
namespace gazebo {
	class PlatformControlPlugin : public ModelPlugin {
		private: ros::NodeHandle* _node_handle;
		private: physics::ModelPtr model;
		private: event::ConnectionPtr updateConnection;
		private: physics::JointPtr _passive_x_joint;
		private: physics::JointPtr _passive_y_joint;
		private: physics::JointPtr _shoulder_joint_x;
		private: physics::JointPtr _shoulder_joint_y;
		private: ros::Publisher _rx_pub;
		private: ros::Publisher _ry_pub;
		private: ros::Publisher _sx_pub;
		private: ros::Publisher _sy_pub;
		private: ros::Publisher _command_sx_pub;
		private: ros::Publisher _command_sy_pub;
		private: ros::Publisher _hx_pub;
		private: ros::Publisher _hy_pub;
		private: std_msgs::Float64 _x_rotation;
		private: std_msgs::Float64 _y_rotation;
		private: std_msgs::Float64 _shoulder_rotation_x;
		private: std_msgs::Float64 _shoulder_rotation_y;
		private: std_msgs::Float64 _hx_rotation;
		private: std_msgs::Float64 _hy_rotation;


		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
			printf("The plugin has been correctly loaded!\n");
			_node_handle = new ros::NodeHandle();
			model = _parent;
			_passive_x_joint = this->model->GetJoint("revolute_joint_x");
			_passive_y_joint = this->model->GetJoint("revolute_joint_y");
			_shoulder_joint_x = this->model->GetJoint("shoulder_joint_x");
			_shoulder_joint_y = this->model->GetJoint("shoulder_joint_y");
			_rx_pub = _node_handle->advertise< std_msgs::Float64 >("/passive_joint/x_rotation", 0);
			_ry_pub = _node_handle->advertise< std_msgs::Float64 >("/passive_joint/y_rotation", 0);
			_sx_pub = _node_handle->advertise< std_msgs::Float64 >("/shoulder_joint/x_rotation", 0);
			_sy_pub = _node_handle->advertise< std_msgs::Float64 >("/shoulder_joint/y_rotation", 0);
			_hx_pub = _node_handle->advertise< std_msgs::Float64 >("/platform_horizontality/x_rotation", 0);
			_hy_pub = _node_handle->advertise< std_msgs::Float64 >("/platform_horizontality/y_rotation", 0);
			_command_sx_pub = _node_handle->advertise< std_msgs::Float64 >("/licasa1/licasa1_shoulder_x_effort_pos_controller/command", 0);	
			_command_sy_pub = _node_handle->advertise< std_msgs::Float64 >("/licasa1/licasa1_shoulder_y_effort_pos_controller/command", 0);
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&PlatformControlPlugin::OnUpdate, this));
		}

		// Called by the world update start event
		public: void OnUpdate() {
			std_msgs::Float64 command_sx;
			std_msgs::Float64 command_sy;

			_x_rotation.data = (_passive_x_joint->Position(0));
			_y_rotation.data = (_passive_y_joint->Position(0));
			_shoulder_rotation_x.data = (_shoulder_joint_x->Position(0));
			_shoulder_rotation_y.data = (_shoulder_joint_y->Position(0));

			_hx_rotation.data = _x_rotation.data + _shoulder_rotation_x.data;
			_hy_rotation.data = _y_rotation.data + _shoulder_rotation_y.data;

			command_sx.data = -_x_rotation.data;
			command_sy.data = -_y_rotation.data;

			_rx_pub.publish(_x_rotation);
			_ry_pub.publish(_y_rotation);
			_sx_pub.publish(_shoulder_rotation_x);
			_sy_pub.publish(_shoulder_rotation_y);
			_command_sx_pub.publish(command_sx);
			_command_sy_pub.publish(command_sy);
			_hx_pub.publish(_hx_rotation);
			_hy_pub.publish(_hy_rotation);
		}

	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(PlatformControlPlugin)
}