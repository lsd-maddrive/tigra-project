#include "car_interface.h"

#include <boost/assign.hpp>


using namespace std;

namespace gazebo
{

Wr8InterfacePlugin::Wr8InterfacePlugin()
{
    target_angle_ = 0.0;
    target_speed_mps_ = 0.0;
    throttle_cmd_ = 0.0;
    current_steering_angle_ = 0.0;
    rollover_ = false;

    x_ = 0; y_ = 0; yaw_ = 0;

    cout << "Wr8 plugin created!" << endl;
}

void Wr8InterfacePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
    // Gazebo initialization
    steer_fl_joint_ = model->GetJoint("joint_left_wheel_1_steer_joint");
    steer_fr_joint_ = model->GetJoint("joint_right_wheel_1_steer_joint");
    wheel_rl_joint_ = model->GetJoint("joint_right_wheel_1__speed_joint");
    wheel_rr_joint_ = model->GetJoint("joint_left_wheel_1_speed_joint");
    wheel_fl_joint_ = model->GetJoint("joint_left_wheel_2_speed_joint");
    wheel_fr_joint_ = model->GetJoint("joint_right_wheel_2_speed_joint");
    footprint_link_ = model->GetLink("base_footprint");

    assert(steer_fl_joint_);
    assert(steer_fr_joint_);
    assert(wheel_rl_joint_);
    assert(wheel_rr_joint_);
    assert(wheel_fl_joint_);
    assert(wheel_fr_joint_);
    assert(footprint_link_);

    // Load SDF parameters
    if (sdf->HasElement("pubTf"))
    {
        sdf->GetElement("pubTf")->GetValue()->Get(pub_tf_);
    }
    else
    {
        pub_tf_ = false;
    }

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
        wheelbase_ = 0.3;
    }

    
    if (sdf->HasElement("wheelRadius"))
    {
        sdf->GetElement("wheelRadius")->GetValue()->Get(wheel_radius_);
    }
    else
    {
        wheel_radius_ = 0.04;
    }

    if (sdf->HasElement("trackWidth"))
    {
        sdf->GetElement("trackWidth")->GetValue()->Get(track_width_);
    }
    else
    {
        track_width_ = 0.23;
    }

    if (sdf->HasElement("tfFreq"))
    {
        sdf->GetElement("tfFreq")->GetValue()->Get(tf_freq_);
    }
    else
    {
        tf_freq_ = 100.0;
    }

    cout << "Wr8 plugin loaded!" << endl;
}

void Wr8InterfacePlugin::OnUpdate(const common::UpdateInfo &info)
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
    updateOdometry();

    driveUpdate();
    steeringUpdate();
}

void Wr8InterfacePlugin::driveUpdate()
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

void Wr8InterfacePlugin::updateCurrentState()
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

void Wr8InterfacePlugin::steeringUpdate()
{
    // Arbitrarily set maximum steering rate to 800 deg/s
    const double max_rate = 800.0 * M_PI / 180.0 * WR8_STEERING_RATIO;
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

void Wr8InterfacePlugin::onCmdVel(const geometry_msgs::Twist& command)
{
    target_angle_ = command.angular.z * WR8_STEERING_RATIO;
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

void Wr8InterfacePlugin::Reset()
{
}

Wr8InterfacePlugin::~Wr8InterfacePlugin()
{
    n_.shutdown();
}

} // namespace gazebo
