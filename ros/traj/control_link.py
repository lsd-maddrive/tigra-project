import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

rospy.init_node('control_link')

def cmd_cb(msg):
    # print(msg)
    print(msg.linear.x, msg.angular.z)

    speed_pub.publish( msg.linear.x * 100 )
    steer_pub.publish( msg.angular.z * 10 )

cmd_sbs = rospy.Subscriber('/cmd_vel', Twist, cmd_cb, queue_size = 1)

speed_pub = rospy.Publisher('quadro/speed_perc', Int8, queue_size=1)
steer_pub = rospy.Publisher('quadro/steer_perc', Int8, queue_size=1)

if __name__ == '__main__':

    rospy.spin()