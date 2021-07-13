import rospy
import actionlib
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, UInt8
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from time import sleep

GOALS_LIST = [
    # x, y, angle, tolerance
    (40, 0, 0, 1)
]

def callback_active():
    rospy.loginfo("Action server is processing the goal")

def callback_done(state, result):
    rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))

def callback_feedback(feedback):
    rospy.loginfo("Feedback:%s" % str(feedback))

    fb = feedback
    # fb.base_position.pose.position.x
    # fb.base_position.pose.orientation.x

if __name__ == '__main__':
    rospy.init_node('control_link')
    print('Ready, go')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    try:

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 40
        goal.target_pose.pose.orientation.w = 1.0

        client.send_goal(goal,
                    active_cb=callback_active,
                    feedback_cb=callback_feedback,
                    done_cb=callback_done)

        print('Set goal')
        print(goal) 

    except:
        client.cancel_goal()
        print('Interrupted')


    rospy.spin()

