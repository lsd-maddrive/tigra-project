import rospy
import actionlib
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, UInt8
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from time import sleep

rospy.init_node('control_link')

def cmd_cb(msg):
    # print(msg)
    print(msg.linear.x, msg.angular.z)

    speed_pub.publish( msg.linear.x * 100 )
    steer_pub.publish( msg.angular.z * -150 )

    cmd = [msg.linear.x * 100, msg.angular.z * -150]

    print('Cmd: ', cmd)

cmd_sbs = rospy.Subscriber('cmd_vel', Twist, cmd_cb, queue_size = 1)

speed_pub = rospy.Publisher('quadro/speed_perc', Int8, queue_size=1)
steer_pub = rospy.Publisher('quadro/steer_perc', Int8, queue_size=1)
mode_pub = rospy.Publisher('quadro/mode_status', UInt8, queue_size=1)

if __name__ == '__main__':

    print('Ready, go')

    for i in range(10):
        mode_pub.publish( 1 )
        sleep( 0.2 )


    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 40
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)

    for i in range(10):
        mode_pub.publish( 2 )
        sleep( 0.2 )

    # goal = PoseStamped()

    # goal.header.frame_id = 'map'
    # goal.header.stamp = rospy.Time.now()
    # # goal.header.seq = 1

    # goal.pose.position.x = 10.0


    # goal_pub.publish( goal )

    print('Set goal')
    print(goal)

    rospy.spin()