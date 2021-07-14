

// ROS
#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Twist.h>


class SteeringSmoothFilter{

public:

    SteeringSmoothFilter(ros::NodeHandle &nh, double alpha) : m_alpha(alpha), m_is_initialized(false) {
        sub_ = nh.subscribe("cmd_vel_raw", 100, &SteeringSmoothFilter::callback, this);
        pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

        ROS_INFO_STREAM("Initialized steering filter with alpha: " << alpha);
    }

    void callback(const geometry_msgs::Twist::ConstPtr &msg) {
        double angle_steering = msg->angular.z;

        if (!m_is_initialized) {
            m_prev_value = angle_steering;
            m_is_initialized = true;
        }

        geometry_msgs::Twist new_msg = *msg;

        double new_steering = m_prev_value*(1-m_alpha) + angle_steering*(m_alpha);
        
        // ROS_INFO_STREAM("Raw: " << angle_steering << " / " << new_steering);

        new_msg.angular.z = m_prev_value = new_steering;
        
        pub_.publish(new_msg);
    }
private:    
    ros::Subscriber sub_;
    ros::Publisher pub_;

    bool m_is_initialized;
    double m_prev_value;
    double m_alpha;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "steering_filter");
    ros::NodeHandle nh;

    float steering_alpha_param;
    ros::param::get("~steering_alpha", steering_alpha_param);

    SteeringSmoothFilter filter(nh, steering_alpha_param);

    ros::spin();
    return 0;
}