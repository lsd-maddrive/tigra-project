#include <iostream>
#include <vector>
using namespace std;

#include <boost/thread/thread.hpp>
#include <boost/assign.hpp>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>

// ROS
#include <ros/ros.h>
#include <tf/tf.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tigra_msgs/TigraState.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// Local includes
#include <robot_layer/odom_model.hpp>

class OdometryConverter
{
public:
    OdometryConverter(
        OdometryBicycleModel &model, 
        ros::NodeHandle &nh, 
        vector<double> &cov_diag, 
        bool publish_tf,
        string &base_frame_id, 
        string &odom_frame_id) : 
    model_(model), last_ts(0), m_publish_tf(publish_tf)
    {
        sub_ = nh.subscribe("state", 100, &OdometryConverter::callback, this);

        pub_.reset(new realtime<arg name="nn" default="3" />           (0)  (0)  (0)  (0)  (0)  (static_cast<double>(cov_diag[5]));
        pub_->msg_.twist.twist.linear.x = 0;
        pub_->msg_.twist.twist.linear.y = 0;
        pub_->msg_.twist.twist.linear.z = 0;
        pub_->msg_.twist.twist.angular.x = 0;
        pub_->msg_.twist.twist.angular.y = 0;
        pub_->msg_.twist.twist.angular.z = 0;
        pub_->msg_.twist.covariance = boost::assign::list_of
                                           (static_cast<double>(cov_diag[0])) (0)  (0)  (0)  (0)  (0)
                                           (0)  (static_cast<double>(cov_diag[1])) (0)  (0)  (0)  (0)
                                           (0)  (0)  (static_cast<double>(cov_diag[2])) (0)  (0)  (0)
                                           (0)  (0)  (0)  (static_cast<double>(cov_diag[3])) (0)  (0)
                                           (0)  (0)  (0)  (0)  (static_cast<double>(cov_diag[4])) (0)
                                           (0)  (0)  (0)  (0)  (0)  (static_cast<double>(cov_diag[5]));

        if (m_publish_tf) {
            tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(nh, "/tf", 100));
            tf_odom_pub_->msg_.transforms.resize(1);
            tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
            tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id;
            tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id;
        }

        // thr_ = new boost::thread(boost::bind(&OdometryConverter::thread_routine, this));
    }

    void callback(const tigra_msgs::TigraState::ConstPtr &msg)
    {
        ROS_INFO_ONCE("Data received");

        double ts = msg->stamp.toSec();

        if (ts < last_ts) {
            model_.resetState();
            ROS_WARN("State reset");
        }

        model_.updateState(msg->angle_steering, msg->rotation_speed, ts);
        publishOdometry();

        last_ts = ts;
    }

    // Disabled now
    void thread_routine()
    {
        ros::Rate loop_rate_(10);
        while (ros::ok()) 
        {
            publishOdometry();
            loop_rate_.sleep();
        }
    }

private:
    double last_ts;
    bool m_publish_tf;

    std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;

    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> pub_;
    ros::Subscriber sub_;

    boost::thread *thr_;

    OdometryBicycleModel &model_;

    void publishOdometry() {
        const double yaw = model_.getYaw();
        const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(yaw));

        const double x = model_.getX();
        const double y = model_.getY();

        const double vx = model_.getVX();
        const double wyaw = model_.getWYaw();

        auto now = ros::Time().now();

        if (pub_->trylock())
        {
            pub_->msg_.header.stamp = now;
            pub_->msg_.pose.pose.position.x = x;
            pub_->msg_.pose.pose.position.y = y;
            pub_->msg_.pose.pose.orientation = orientation;
            pub_->msg_.twist.twist.linear.x = vx;
            pub_->msg_.twist.twist.angular.z = wyaw;
            pub_->unlockAndPublish();
        }

        if (m_publish_tf && tf_odom_pub_->trylock()) 
        {
            geometry_msgs::TransformStamped &odom_frame = tf_odom_pub_->msg_.transforms[0];
            odom_frame.header.stamp = now;
            odom_frame.transform.translation.x = x;
            odom_frame.transform.translation.y = y;
            odom_frame.transform.rotation = orientation;
            tf_odom_pub_->unlockAndPublish();
        }
    }
};

class CommandConverter
{
public:
    CommandConverter(ros::NodeHandle &nh, double wheel_radius) : wheel_radius_(wheel_radius)
    {
        sub_ = nh.subscribe("cmd_vel", 100, &CommandConverter::callback, this);
        pub_ = nh.advertise<tigra_msgs::TigraState>("state_cmd", 100);

        mps2rps_ = 1/wheel_radius_;
    }

    void callback(const geometry_msgs::Twist::ConstPtr &msg) {
        msg_.stamp = ros::Time::now();
        msg_.angle_steering = msg->angular.z;
        msg_.rotation_speed = msg->linear.x * mps2rps_;
        pub_.publish(msg_);
    }

private:
    double wheel_radius_;
    double mps2rps_;
    
    ros::Subscriber sub_;
    ros::Publisher pub_;

    tigra_msgs::TigraState msg_;
};

// Used for "cout << vector"
template <typename T>
std::ostream &operator<<(std::ostream &out, const std::vector<T> &v)
{
    if (!v.empty())
    {
        out << '[';
        std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
        out << "\b\b]";
    }
    return out;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_layer");

    ros::NodeHandle nh;

    ROS_INFO_STREAM("Robot layer started");

    double wheelbase_param;
    ros::param::get("~model_props/wheelbase", wheelbase_param);

    double wheel_radius_param;
    ros::param::get("~model_props/wheel_radius", wheel_radius_param);

    ROS_INFO_STREAM("Wheelbase: " << wheelbase_param);
    ROS_INFO_STREAM("Wheel radius: " << wheel_radius_param);

    vector<double> odom_cov_diag_param;
    ros::param::get("~main_config/odometry/cov_diag", odom_cov_diag_param);
    string base_frame_id_param;
    ros::param::get("~main_config/odometry/base_frame_id", base_frame_id_param);
    string odom_frame_id_param;
    ros::param::get("~main_config/odometry/frame_id", odom_frame_id_param);

    bool publish_tf_param;
    ros::param::get("~publish_tf", publish_tf_param);

    // ROS_INFO_STREAM("Odometry covariance daigonal: " << odom_cov_diag_param);
    ROS_INFO_STREAM("Frame IDs - Base: " << base_frame_id_param << " Odom: " << odom_frame_id_param);
    ROS_INFO_STREAM("Publish TF: " << publish_tf_param);

    OdometryBicycleModel odom_model(wheel_radius_param, wheelbase_param);
    OdometryConverter odom_conv(odom_model, nh, odom_cov_diag_param, publish_tf_param, base_frame_id_param, odom_frame_id_param);
    CommandConverter cmd_conv(nh, wheel_radius_param);

    ros::spin();
    return 0;
}
