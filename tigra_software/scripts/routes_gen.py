#!/usr/bin/env python
import rospy
import actionlib
import sys
import tty
import termios
import rospkg

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class RouteGenerator:
    def __init__(self) -> None:
        rospy.init_node('routes_generator', anonymous=True)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) # to cancel goals

        rospy.loginfo('Ok, lets go!')
        self.result_route='dist_tol: 1\npoints:\n'

        rospack = rospkg.RosPack()
        path_to_cfg_folder = f"{rospack.get_path('tigra_software')}/config/routes/new_route.yaml"
        self.text_file = open(path_to_cfg_folder, "w")


    def getch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def callback(self, data):
        my_string=str(data.pose)
        splitted_string=my_string.split('\n')
        rospy.loginfo('Point received!')
        self.result_route += '  - n: 1\n'
        for x in splitted_string:
            self.result_route += '    ' + str(x) + '\n'
        
    def spin(self):
        while not rospy.is_shutdown():
            kk = self.getch()
            if kk == 'q':
                self.text_file.write(self.result_route)
                self.text_file.close()
                self.client.cancel_goal()
                self.client.cancel_all_goals() # Try both
                rospy.loginfo(f'God bye! Final route is\n {self.result_route}')
                exit(0)


if __name__ == '__main__':
    route_gen=RouteGenerator()
    route_gen.spin()