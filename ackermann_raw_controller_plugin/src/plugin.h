#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tigra_msgs/TigraState.h>

// Control parameters
#define STEER_P_RATE 10.0

namespace gazebo
{
    class TigraPlugin : public ModelPlugin
    {
    public:
        TigraPlugin();
        virtual ~TigraPlugin();

    protected:
        virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
        virtual void Reset();

    private:
        void onStateCmd(const tigra_msgs::TigraState &command);
        void updateCurrentState();
        void updateStatePub(double time_step);

        void twistTimerCallback(const ros::TimerEvent &event);
        void tfTimerCallback(const ros::TimerEvent &event);
        void OnUpdate(const common::UpdateInfo &info);

        void driveUpdate();
        void steeringUpdate(double time_step);

        void stopWheels();

        template <typename T>
        T clip(const T &n, const T &lower, const T &upper)
        {
            return std::max(lower, std::min(n, upper));
        }

        ros::NodeHandle n_;

        event::ConnectionPtr update_connection_;
        physics::JointPtr steer_fl_joint_;
        physics::JointPtr steer_fr_joint_;
        physics::JointPtr wheel_rl_joint_;
        physics::JointPtr wheel_rr_joint_;
        common::Time last_update_time_;

        ros::Subscriber sub_state_cmd_;
        std::shared_ptr<realtime_tools::RealtimePublisher<tigra_msgs::TigraState>> state_pub_;

        std::string odom_frame_id_;
        std::string base_frame_id_;

        double max_steering_angle_deg_;
        double max_steering_angle_;

        common::Time last_state_pub_time_;

        double publish_frequency_;
        double publish_period_;

        // SDF parameters
        std::string robot_name_;
        double wheelbase_;
        double track_width_;

        // Steering values
        double right_angle_;
        double left_angle_;
        double current_steering_angle_;

        double cur_virtual_steering_rad_;
        double cur_virtual_speed_rps_;

        // Targets
        double target_steer_rad_;
        double target_speed_rps_;

        double throttle_cmd_;
        ros::Time throttle_stamp_;
    };

    GZ_REGISTER_MODEL_PLUGIN(TigraPlugin)

}
