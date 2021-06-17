#include "plugin.h"

#include <boost/assign.hpp>

using namespace std;

namespace gazebo
{

    TigraPlugin::TigraPlugin() : target_steer_rad_(0), target_speed_rps_(0), current_steering_angle_(0)
    {
        cout << "TigraPlugin plugin created!" << endl;
    }

    TigraPlugin::~TigraPlugin()
    {
        n_.shutdown();
    }

    void TigraPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        // auto joints = model->GetJoints();
        // for (int i = 0; i < model->GetJointCount(); i++)
        // {
        //     ROS_INFO_STREAM("Joint: " << joints[i]->GetName());
        // }

        if (model->GetJointCount() == 0)
        {
            std::cerr << "Invalid joint count, plugin not loaded\n";
            return;
        }

        // Gazebo initialization
        steer_fl_joint_ = model->GetJoint("front_left_wheel_steer_joint");
        steer_fr_joint_ = model->GetJoint("front_right_wheel_steer_joint");

        wheel_rl_joint_ = model->GetJoint("rear_left_wheel_speed_joint");
        wheel_rr_joint_ = model->GetJoint("rear_right_wheel_speed_joint");

        assert(steer_fl_joint_);
        assert(steer_fr_joint_);
        assert(wheel_rl_joint_);
        assert(wheel_rr_joint_);

        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM_NAMED("robot", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                                << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        if (sdf->HasElement("robotName"))
        {
            robot_name_ = sdf->GetElement("robotName")->Get<std::string>();
        }
        else
        {
            robot_name_ = std::string("");
        }

        // Parameters required to compute correct state
        if (sdf->HasElement("wheelbase"))
        {
            sdf->GetElement("wheelbase")->GetValue()->Get(wheelbase_);
        }
        else
        {
            wheelbase_ = 1.0;
        }

        if (sdf->HasElement("trackWidth"))
        {
            sdf->GetElement("trackWidth")->GetValue()->Get(track_width_);
        }
        else
        {
            track_width_ = 0.8;
        }

        publish_frequency_ = 20;

        max_steering_angle_deg_ = 25;
        max_steering_angle_ = max_steering_angle_deg_ / 180 * M_PI;

        publish_period_ = 1. / publish_frequency_;

        update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&TigraPlugin::OnUpdate, this, _1));

        steer_fl_joint_->SetParam("fmax", 0, 99999.0);
        steer_fr_joint_->SetParam("fmax", 0, 99999.0);

        // ROS initialization
        n_ = ros::NodeHandle(robot_name_);

        sub_state_cmd_ = n_.subscribe("state_cmd", 1, &TigraPlugin::onStateCmd, this);

        // Setup odometry realtime publisher + odom message constant fields
        state_pub_.reset(new realtime_tools::RealtimePublisher<tigra_msgs::TigraState>(n_, "state", 100));

        cout << "TigraPlugin plugin loaded!" << endl;
    }

    void TigraPlugin::OnUpdate(const common::UpdateInfo &info)
    {
        if (last_update_time_ == common::Time(0))
        {
            last_update_time_ = info.simTime;
            return;
        }

        double time_step_ = (info.simTime - last_update_time_).Double();
        last_update_time_ = info.simTime;

        updateCurrentState();
        updateStatePub(time_step_);

        driveUpdate();
        steeringUpdate(time_step_);
    }

    void TigraPlugin::updateStatePub(double time_step)
    {
        /*** Update publishers ***/
        if (last_state_pub_time_ + publish_period_ > last_update_time_)
        {
            return;
        }

        last_state_pub_time_ += publish_period_;

        if (state_pub_->trylock())
        {
            state_pub_->msg_.angle_steering = cur_virtual_steering_rad_;
            state_pub_->msg_.rotation_speed = cur_virtual_speed_rps_;
            state_pub_->msg_.stamp = ros::Time::now();
            state_pub_->unlockAndPublish();
        }
    }

    void TigraPlugin::updateCurrentState()
    {
        double t_cur_lsteer = tan(steer_fl_joint_->Position(0));
        double t_cur_rsteer = tan(steer_fr_joint_->Position(0));

        // std::atan(wheelbase_ * std::tan(steering_angle)/std::abs(wheelbase_ + it->lateral_deviation_ * std::tan(steering_angle)));

        double virt_lsteer = atan(wheelbase_ * t_cur_lsteer / (wheelbase_ + t_cur_lsteer * 0.5 * track_width_));
        double virt_rsteer = atan(wheelbase_ * t_cur_rsteer / (wheelbase_ - t_cur_rsteer * 0.5 * track_width_));

        // ROS_INFO_STREAM( "Steerings: " << virt_lsteer << " / " << virt_rsteer );

        cur_virtual_steering_rad_ = (virt_lsteer + virt_rsteer) / 2;
        cur_virtual_speed_rps_ = (wheel_rl_joint_->GetVelocity(0) + wheel_rr_joint_->GetVelocity(0)) / 2;

        // ROS_INFO_STREAM( "Estimated state: " << cur_virtual_steering_rad_ << " / " << cur_virtual_speed_rps_ );
    }

    void TigraPlugin::driveUpdate()
    {
        double cur_right_rspeed_rps = wheel_rr_joint_->GetVelocity(0);
        double cur_left_rpeed_rps = wheel_rl_joint_->GetVelocity(0);

        double radius = wheelbase_ / tan(cur_virtual_steering_rad_);

        double ref_right_rspeed = target_speed_rps_ * (1 + (0.5 * track_width_ / radius));
        double ref_left_rspeed = target_speed_rps_ * (1 - (0.5 * track_width_ / radius));

        // double right_rspeed_error = ref_right_rspeed - cur_right_rspeed_rps;
        // double left_rspeed_error = ref_left_rspeed - cur_left_rpeed_rps;

        wheel_rl_joint_->SetVelocity(0, ref_right_rspeed);
        wheel_rr_joint_->SetVelocity(0, ref_left_rspeed);
    }

    void TigraPlugin::steeringUpdate(double time_step)
    {
        // Compute Ackermann steering angles for each wheel
        double t_alph = tan(target_steer_rad_);
        double left_steer = atan(wheelbase_ * t_alph / (wheelbase_ - 0.5 * track_width_ * t_alph));
        double right_steer = atan(wheelbase_ * t_alph / (wheelbase_ + 0.5 * track_width_ * t_alph));

        steer_fl_joint_->SetParam("vel", 0, STEER_P_RATE * (left_steer - steer_fl_joint_->Position(0)));
        steer_fr_joint_->SetParam("vel", 0, STEER_P_RATE * (right_steer - steer_fr_joint_->Position(0)));
    }

    void TigraPlugin::onStateCmd(const tigra_msgs::TigraState &cmd)
    {
        target_steer_rad_ = clip((double)cmd.angle_steering, -max_steering_angle_, max_steering_angle_);
        target_speed_rps_ = cmd.rotation_speed;
    }

    void TigraPlugin::Reset()
    {
    }
} // namespace gazebo
