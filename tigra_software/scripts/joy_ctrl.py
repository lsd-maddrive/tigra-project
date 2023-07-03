#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import numpy as np
import time



# BUTTONS_NUM={   "A":0,
#                 "B":1,
#                 "X":2,
#                 "Y":3,
#                 "LB":4,
#                 "RB":5,
#                 "BACK":6,
#                 "START":7,
#                 "LIVE":8,
#                 "LS":9,
#                 "RS":10}


class Controller():
    def __init__(self) -> None:
        self.axis=self.TwoDirectionVelocity
        self.buttons_status=self.ButtonsStatus
        self.command_state = Twist()
        self.buttons_msg= Int32MultiArray()

        rospy.init_node("control_link")
        rospy.Subscriber("joy", Joy, self.joy_cb, queue_size=5)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Wating for move_base server.....")       
        print('aboba')
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        self.btn_pub = rospy.Publisher("button_pressed", Int32MultiArray, queue_size=5)
        self.rate = 5
       
        self.debug_enabled = rospy.get_param("~debug", False)
        if self.debug_enabled:
            rospy.loginfo("Debug enabled")
        forward_speed_limit_mps = rospy.get_param("~speed/frwd_limit", 1)
        backward_speed_limit_mps = rospy.get_param("~speed/bkwrd_limit", -1)
        steer_limit_deg = rospy.get_param("~steer/limit", 25)
        steer_limit_rad = np.deg2rad(steer_limit_deg)

        self.linear_vel = self.axis(
            min_value=backward_speed_limit_mps, max_value=forward_speed_limit_mps
        )
        self.angular_pos = self.axis(
            min_value=-steer_limit_rad, max_value=steer_limit_rad
        )
        self.buttons = self.buttons_status()
        rospy.loginfo("Ready, go!")


    def joy_cb(self, msg):

        if self.debug_enabled:
            self.show_clicked(msg)

        ## last message timestamp
        # self.linear_vel.last_stamp=msg.header.stamp.secs
        self.linear_vel.last_stamp=time.time()

        self.angular_pos.set_relative(msg.axes[0])
        self.linear_vel.set_relative(msg.axes[4])
        self.buttons.set_buttons(msg.buttons)


    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
    
    def button_handler(self,button_arr):
        if button_arr[1] == 1:
            self.client.cancel_goal()
            self.client.cancel_all_goals() # Try both
            rospy.loginfo("Goal cancelled")


    def update(self):
       
        # TODO Communication loss protection
        # delta =time.time() - self.linear_vel.last_stamp
        # print(f'delta is {delta}')
        # if time.time() - self.linear_vel.last_stamp > 5:
        #     self.command_state.linear.x = 0
        #     # print('CONNECTION LOST!!')
        # else:
        #     self.command_state.linear.x = self.linear_vel.get_velocity()
            
        self.command_state.linear.x = self.linear_vel.get_velocity()
        self.command_state.angular.z = self.angular_pos.get_velocity()
        self.buttons_msg.data=self.buttons.get_buttons()
        self.button_handler(self.buttons_msg.data)

        # if self.buttons_msg.data[1] == 1:
        #     self.client.cancel_goal()
        #     self.client.cancel_all_goals() # Try both
        #     rospy.loginfo("Goal cancelled")

        self.cmd_pub.publish(self.command_state)
        self.btn_pub.publish(self.buttons_msg)


    def show_clicked(self,msg):
        button_names = ["A","B","X","Y",
                    "LB","RB","BACK","START",
                    "LIVE","LS","RS",
        ]
        axes_names = ["Lstick horz","Lstick vert",
                    "LT","Rstick horz",
                    "Rstick vert","RT","?","?",
        ]      
        print("Buttons:")
        for i in range(len(msg.buttons)):
            if msg.buttons[i] != 0:
                print("\t" + button_names[i])

        print("Axes:")
        for i in range(len(msg.axes)):
            print("\t%s: %.2f" % (axes_names[i], msg.axes[i]))

        # print(f'timestamp diif is {int(time.time()) - self.linear_vel.last_stamp}')
        
    # def GoalHandler

    class TwoDirectionVelocity:
        def __init__(self, min_value, max_value, zero_point=0):
            assert min_value < zero_point < max_value

            self._low_ratio = zero_point - min_value
            self._high_ratio = max_value - zero_point

            self._zero_point = zero_point
            self._velocity = 0
            self.last_stamp = 0

        def set_relative(self, ratio):
            ratio = np.clip(ratio, -1, 1)

            if ratio < 0:
                self._velocity = self._zero_point + ratio * self._low_ratio
            else:
                self._velocity = self._zero_point + ratio * self._high_ratio

        def get_velocity(self):
            return self._velocity
        
    class ButtonsStatus:
        def __init__(self) -> None:
            self._pressed_buttons=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        def set_buttons(self,msg):
            self._pressed_buttons=msg
            
        def get_buttons(self):
            return self._pressed_buttons
        

def update_command_state():
    global command_state, cmd_pub, has_control_commands

    command_state.linear.x = linear_vel.get_velocity()
    command_state.angular.z = angular_pos.get_velocity()

    if has_control_commands or (command_state.linear.x != 0) or (command_state.angular.z != 0):
        cmd_pub.publish(command_state)
        has_control_commands = (command_state.linear.x != 0) or (command_state.angular.z != 0)


if __name__ == "__main__":
    try:
        controller=Controller()
        controller.spin()
    except rospy.ROSInterruptException:
        print('Exception catched!')
