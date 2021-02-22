#include "plugin.h"

#include <boost/assign.hpp>


using namespace std;

namespace gazebo
{

TigraPlugin::TigraPlugin()
{
    target_angle_ = 0.0;
    target_speed_mps_ = 0.0;
    throttle_cmd_ = 0.0;
    current_steering_angle_ = 0.0;
    rollover_ = false;

    x_ = 0; y_ = 0; yaw_ = 0;

    cout << "TigraPlugin plugin created!" << endl;
}

void TigraPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
    // Gazebo initialization
    steer_fl_joint_ = model->GetJoint("joint_left_wheel_1_steer_joint");
    steer_fr_joint_ = model->GetJoint("joint_right_wheel_1_steer_joint");
    wheel_rl_joint_ = model->GetJoint("joint_left_wheel_1_speed_joint");
    wheel_rr_joint_ = model->GetJoint("joint_right_wheel_1_speed_joint");
    wheel_fl_joint_ = model->GetJoint("joint_left_wheel_2_speed_joint");
    wheel_fr_joint_ = model->GetJoint("joint_right_wheel_2_speed_joint");

    assert(steer_fl_joint_);
    assert(steer_fr_joint_);
    assert(wheel_rl_joint_);
    assert(wheel_rr_joint_);
    assert(wheel_fl_joint_);
    assert(wheel_fr_joint_);

    if (sdf->HasElement("robotName"))
    {
        sdf::ParamPtr sdf_robot_name = sdf->GetElement("robotName")->GetValue();
        if (sdf_robot_name)
        {
            sdf_robot_name->Get(robot_name_);
        }
        else
        {
            robot_name_ = std::string("");
        }
    }
    else
    {
        robot_name_ = std::string("");
    }


    if (sdf->HasElement("maxSteerRad"))
    {
        sdf->GetElement("maxSteerRad")->GetValue()->Get(max_steer_rad_);
    }
    else
    {
        max_steer_rad_ = M_PI * 30 / 180;
    }

    if (sdf->HasElement("wheelbase"))
    {
        sdf->GetElement("wheelbase")->GetValue()->Get(wheelbase_);
    }
    else
    {
        wheelbase_ = 1.0;
    }

    
    if (sdf->HasElement("wheelRadius"))
    {
        sdf->GetElement("wheelRadius")->GetValue()->Get(wheel_radius_);
    }
    else
    {
        wheel_radius_ = 0.25;
    }

    if (sdf->HasElement("trackWidth"))
    {
        sdf->GetElement("trackWidth")->GetValue()->Get(track_width_);
    }
    else
    {
        track_width_ = 0.8;
    }

    if (sdf->HasElement("tfFreq"))
    {
        sdf->GetElement("tfFreq")->GetValue()->Get(tf_freq_);
    }
    else
    {
        tf_freq_ = 10.0;
    }

    /* TODO - read from SDF */
    odom_frame_id_ = "odom";
    base_frame_id_ = "base_footprint";
    publish_period_ = 1. / 100;

    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&TigraPlugin::OnUpdate, this, _1));

    mps2rpm = 60 / wheel_radius_ / (2*M_PI);
    mps2rps = 1.0 / wheel_radius_;

    steer_fl_joint_->SetParam("fmax", 0, 99999.0);
    steer_fr_joint_->SetParam("fmax", 0, 99999.0);

    // ROS initialization
    n_ = ros::NodeHandle(robot_name_);

    sub_vel_cmd_ = n_.subscribe("cmd_vel", 1, &TigraPlugin::onCmdVel, this);

    cout << "TigraPlugin plugin loaded!" << endl;
}


void TigraPlugin::OnUpdate(const common::UpdateInfo &info)
{
    if (last_update_time_ == common::Time(0))
    {
        last_update_time_ = info.simTime;
        return;
    }

    time_step_ = (info.simTime - last_update_time_).Double();
    last_update_time_ = info.simTime;

    // twistStateUpdate();
    updateCurrentState();

    driveUpdate();
    steeringUpdate();
}

void TigraPlugin::driveUpdate()
{
    double ref_rotation_speed_rps = target_speed_mps_ * mps2rps;

    double cur_right_rspeed_rps = wheel_rr_joint_->GetVelocity(0);
    double cur_left_rpeed_rps = wheel_rl_joint_->GetVelocity(0);

    double radius = wheelbase_ / tan(cur_virtual_steering_rad_) ;

    double ref_right_rspeed = ref_rotation_speed_rps * (1 + (0.5 * track_width_ / radius));
    double ref_left_rspeed = ref_rotation_speed_rps * (1 - (0.5 * track_width_ / radius));

    // ROS_INFO_STREAM( "Target speeds: " << ref_left_rspeed << " / " << ref_right_rspeed );

    double right_rspeed_error = ref_right_rspeed - cur_right_rspeed_rps;
    double left_rspeed_error = ref_left_rspeed - cur_left_rpeed_rps;

    wheel_rl_joint_->SetVelocity(0, ref_right_rspeed);
    wheel_rr_joint_->SetVelocity(0, ref_left_rspeed);   
}

void TigraPlugin::updateCurrentState()
{
    double t_cur_lsteer = tan(steer_fl_joint_->Position(0));
    double t_cur_rsteer = tan(steer_fr_joint_->Position(0));

    // std::atan(wheelbase_ * std::tan(steering_angle)/std::abs(wheelbase_ + it->lateral_deviation_ * std::tan(steering_angle)));

    double virt_lsteer = atan( wheelbase_ * t_cur_lsteer / ( wheelbase_ + t_cur_lsteer * 0.5 * track_width_ ) );
    double virt_rsteer = atan( wheelbase_ * t_cur_rsteer / ( wheelbase_ - t_cur_rsteer * 0.5 * track_width_ ) );

    // ROS_INFO_STREAM( "Steerings: " << virt_lsteer << " / " << virt_rsteer );

    cur_virtual_steering_rad_ = (virt_lsteer + virt_rsteer) / 2;

    cur_virtual_speed_rps_ = (wheel_rl_joint_->GetVelocity(0) + wheel_rr_joint_->GetVelocity(0)) / 2;

    // ROS_INFO_STREAM( "Estimated state: " << cur_virtual_steering_rad_ << " / " << cur_virtual_speed_rps_ );
}

void TigraPlugin::steeringUpdate()
{
    // Arbitrarily set maximum steering rate to 800 deg/s
    const double max_rate = 800.0 * M_PI / 180.0 * TIGRA_STEERING_RATIO;
    double max_inc = time_step_ * max_rate;

    // if ((target_angle_ - current_steering_angle_) > max_inc)
    // {
    //     current_steering_angle_ += max_inc;
    // }
    // else if ((target_angle_ - current_steering_angle_) < -max_inc)
    // {
    //     current_steering_angle_ -= max_inc;
    // }

    // ROS_INFO_STREAM( "Target steering: " << target_angle_ );

    // Compute Ackermann steering angles for each wheel
    double t_alph = tan(target_angle_);
    double left_steer = atan(wheelbase_ * t_alph / (wheelbase_ - 0.5 * track_width_ * t_alph));
    double right_steer = atan(wheelbase_ * t_alph / (wheelbase_ + 0.5 * track_width_ * t_alph));

#if GAZEBO_MAJOR_VERSION >= 9
    steer_fl_joint_->SetParam("vel", 0, STEER_P_RATE * (left_steer - steer_fl_joint_->Position(0)));
    steer_fr_joint_->SetParam("vel", 0, STEER_P_RATE * (right_steer - steer_fr_joint_->Position(0)));
#else
    steer_fl_joint_->SetParam("vel", 0, STEER_P_RATE * (left_steer - steer_fl_joint_->GetAngle(0).Radian()));
    steer_fr_joint_->SetParam("vel", 0, STEER_P_RATE * (right_steer - steer_fr_joint_->GetAngle(0).Radian()));
#endif
}

void TigraPlugin::onCmdVel(const geometry_msgs::Twist& command)
{
    target_angle_ = command.angular.z * TIGRA_STEERING_RATIO;
    if (target_angle_ > max_steer_rad_)
    {
        target_angle_ = max_steer_rad_;
    }
    else if (target_angle_ < -max_steer_rad_)
    {
        target_angle_ = -max_steer_rad_;
    }

    target_speed_mps_ = command.linear.x;
}

void TigraPlugin::Reset()
{
}

TigraPlugin::~TigraPlugin()
{
    n_.shutdown();
}


} // namespace gazebo