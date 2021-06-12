#! /usr/bin/env python
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import rospy

rospy.init_node('odom_pub')

odom_pub=rospy.Publisher ('/my_odom', Odometry, queue_size=10)

rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom=Odometry()
header = Header()
header.frame_id='/py_odom'
odom.child_frame_id='/base_footprint'

model = GetModelStateRequest()
model.model_name='robot'

r = rospy.Rate(2)

while not rospy.is_shutdown():
    result = get_model_srv(model)

    odom.pose.pose = result.pose
    odom.twist.twist = result.twist

    header.stamp = rospy.Time.now()
    odom.header = header

    odom_pub.publish(odom)

    r.sleep()