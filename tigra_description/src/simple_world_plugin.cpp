#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>

#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

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
		if (sdf->HasElement("_joint"))
		{
			wheel_joint = model->GetJoint("joint_right_wheel_2_speed_joint");
			cout << "********************Joint received********************" << endl;
		}
		else
		{
			cout << "********************Joint don't received********************" << endl;
		}
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this));
	}
	void OnUpdate()
    	{

      			wheel_joint->SetVelocity(0, speed);
    	}
	private: 
	double speed=0;
	physics::ModelPtr model;
	physics::JointPtr wheel_joint;
	sdf::ElementPtr sdf;
	std::string roboname;	
    	ros::NodeHandle N_H;
	ros::Subscriber sub;
	event::ConnectionPtr updateConnection;
};
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
