#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>

using namespace std;

namespace gazebo
{

class ModelPush : public ModelPlugin
{
	public: 
	ModelPush() : ModelPlugin()
  	{
  	    
		cout << "********************Plugin is creating********************" << endl;	
  	}
	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
		this->model = _parent;
		this->sdf = _sdf;
    //steer_fl_joint_ = model->GetJoint("joint_left_wheel_1_steer_joint");
    //steer_fr_joint_ = model->GetJoint("joint_right_wheel_1_steer_joint");
    wheel_rl_joint_ = model->GetJoint("joint_right_wheel_1_speed_joint");
    wheel_rr_joint_ = model->GetJoint("joint_left_wheel_1_speed_joint");
    wheel_fl_joint_ = model->GetJoint("joint_left_wheel_2_speed_joint");
    wheel_fr_joint_ = model->GetJoint("joint_right_wheel_2_speed_joint");

    //assert(steer_fl_joint_);
    //assert(steer_fr_joint_);
    assert(wheel_rl_joint_);
    assert(wheel_rr_joint_);
    assert(wheel_fl_joint_);
    assert(wheel_fr_joint_);

    
		if (sdf->HasElement("robotName"))
		{
			roboname = sdf->Get<std::string>("robotName");
			cout << "********************Name received********************" << endl;
			cout << "His name is " << roboname << endl;
		}
		else
		{
			cout << "********************Name don't received********************" << endl;
		}
		updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this));
		nh = ros::NodeHandle(roboname);
    		cmd_vel_sub_ = nh.subscribe("cmd_vel", 1, &ModelPush::CmdVel, this);
	}
	
	
	//void ModelPlugin::steeringUpdate()
	//{
	//target_angle_ = 0.0;
    //double t_alph = tan(target_angle_);
    //double left_steer = atan(1.0 * t_alph / (1.0 - 0.5 * 0.8 * t_alph));
    //double right_steer = atan(1.0 * t_alph / (1.0 + 0.5 * 0.8 * t_alph));
    //#if GAZEBO_MAJOR_VERSION >= 9
    //steer_fl_joint_->SetParam("vel", 0, (left_steer - steer_fl_joint_->Position(0)));
    //steer_fr_joint_->SetParam("vel", 0, (right_steer - steer_fr_joint_->Position(0)));
	//#else
    //steer_fl_joint_->SetParam("vel", 0, (left_steer - steer_fl_joint_->GetAngle(0).Radian()));
    //steer_fr_joint_->SetParam("vel", 0, (right_steer - steer_fr_joint_->GetAngle(0).Radian()));
	//#endif
	//}


	void OnUpdate()
    	{	
    wheel_rl_joint_->SetVelocity(0, speed);
    wheel_rr_joint_->SetVelocity(0, speed);
    wheel_fl_joint_->SetVelocity(0, speed);
    wheel_fr_joint_->SetVelocity(0, speed);
    	}
	void CmdVel(const geometry_msgs::Twist& command)
    	{
    	 	target_angle_ = command.angular.z;
			speed = command.linear.x;
      	}
	~ModelPush()
	{
		nh.shutdown();
	}
	private: 
	double speed=0;
	double target_angle_ = 0;
	//double lin_speed;
	physics::ModelPtr model;
	sdf::ElementPtr sdf;
	std::string roboname;
	//physics::ContactManager *contactManager;	
    ros::NodeHandle nh;
	event::ConnectionPtr updateConnection;
	ros::Subscriber cmd_vel_sub_;
	//physics::JointPtr steer_fl_joint_;
	//physics::JointPtr steer_fr_joint_;
	physics::JointPtr wheel_rl_joint_;
	physics::JointPtr wheel_rr_joint_;
	physics::JointPtr wheel_fl_joint_;
	physics::JointPtr wheel_fr_joint_;

};
  
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
