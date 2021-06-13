#include <iostream>
using namespace std;

#include <chrono>
using namespace std::chrono;

// ----------- Here code for uC starts
#include <ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Int8.h>

void speed_cb(const std_msgs::Int8 &msg)
{
    cout << "Received message: " << int(msg.data) << endl;
}

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Int8> topic_in_sample("in_sample", &speed_cb);

std_msgs::String hello_msg;
ros::Publisher topic_out_sample("out_sample", &hello_msg);

int main(int argc, char **argv)
{
    /* ROS setup */
    nh.initNode();
    // Set spin in milliseconds (really units depend on what time() in hardware implementation returns)
    nh.setSpinTimeout(20);

    /* ROS publishers */
    nh.advertise(topic_out_sample);

    /* ROS subscribers */
    nh.subscribe(topic_in_sample);

    // Yeah, this is not from uC (time), but you can implement yours
    // Better use publication from required places of code out of this file with functions
    // Like publishOdometryData()
    high_resolution_clock::time_point send_time = high_resolution_clock::now();

    while (1)
    {
        high_resolution_clock::time_point curr_time = high_resolution_clock::now();
        chrono::duration<double, std::milli> time_span = curr_time - send_time;

        // Send the message every second
        if (time_span.count() > 1000)
        {
            hello_msg.data = "hello";
            topic_out_sample.publish(&hello_msg);

            send_time = high_resolution_clock::now();
        }
        nh.spinOnce();
    }

    return 0;
}
