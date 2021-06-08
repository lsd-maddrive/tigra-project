#!/usr/bin/env python
from nav_msgs.msg import Odometry
import rospy
import tf

def handle_tigra_pose(msg):
    # print(msg.pose.pose.position.x)
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     tf.transformations.quaternion_from_euler(msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z),
                     rospy.Time.now(),
                     "base_footprint",
                     "py_odom")



if __name__ == '__main__':
    rospy.init_node('tf_my_bot')

    rospy.Subscriber('/my_odom',
                     Odometry,
                     handle_tigra_pose
                     )
    rospy.spin()