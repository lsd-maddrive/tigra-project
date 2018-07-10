#!/usr/bin/env python
from __future__ import print_function

from Tkinter import *
from PIL import Image
from PIL import ImageTk

import numpy as np
import matplotlib, sys
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import time

import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import Image, Range, JointState

import cv2
from cv_bridge import CvBridge, CvBridgeError

root = Tk()

control_canvas_width = 400
control_canvas_height = 400

manual_control_steering_value = 0
manual_control_velocity_value = 0

manual_control_enable_publish = False
manual_control_steering_set_only = False
manual_control_velocity_set_only = False

manual_control_publish_period_ms = 100

manual_control_enabled = True

# ---------------------------------------------------------------------------

def manual_publish_control():
    ros_controller_set_control(manual_control_velocity_value, manual_control_steering_value)

    if manual_control_enable_publish:
        root.after(manual_control_publish_period_ms, manual_publish_control)

# ---------------------------------------------------------------------------

def manual_control_callback_mouse_released(event):
    global manual_control_velocity_value, manual_control_steering_value, \
            manual_control_enable_publish, \
            manual_control_steering_set_only, manual_control_velocity_set_only

    manual_control_velocity_value = 0
    manual_control_steering_value = 0
    # manual_control_enable_publish = False
    manual_control_velocity_set_only = False
    manual_control_steering_set_only = False
    # print ('Released')

def manual_conrtol_callback_mouse_motion(event):
    global manual_control_steering_value, manual_control_velocity_value, \
            manual_control_enable_publish
    
    if not manual_control_enabled:
        return

    if not manual_control_velocity_set_only:
        x = np.clip(event.x, 0, control_canvas_width)
        manual_control_steering_value = (float(x) / control_canvas_width - 0.5) * 2 * steering_abs_limit

    if not manual_control_steering_set_only:
        y = np.clip(event.y, 0, control_canvas_height)
        manual_control_velocity_value = (float(y) / control_canvas_height - 0.5) * -2 * velocity_abs_limit

    if not manual_control_enable_publish:
        root.after(manual_control_publish_period_ms, manual_publish_control)

    manual_control_enable_publish = True
    # print ('Motion', x, y)

def manual_control_callback_ctrl_mouse_click(event):
    global manual_control_steering_set_only

    manual_control_steering_set_only = True

def manual_control_callback_shift_mouse_click(event):
    global manual_control_velocity_set_only

    manual_control_velocity_set_only = True

# ---------------------------------------------------------------------------

def init_gui():
    global rangefinder1_text, rangefinder2_text, \
            rangefinder3_text, rangefinder4_text, \
            manual_control_canvas_wgt, toggle_control_state_btn, \
            lpf_rangefinder_coefficient

    manual_control_canvas_wgt = Canvas(root, width=control_canvas_width, height=control_canvas_height, \
                                        highlightbackground='red', highlightthickness=3)
    manual_control_canvas_wgt.grid(row=1, column=0, columnspan=4)
    manual_control_canvas_wgt.create_line(control_canvas_width/2, 0, control_canvas_width/2, control_canvas_height, fill='blue')
    manual_control_canvas_wgt.create_line(0, control_canvas_height/2, control_canvas_width, control_canvas_height/2, fill='blue')

    manual_control_canvas_wgt.bind('<ButtonPress-1>', manual_conrtol_callback_mouse_motion)
    manual_control_canvas_wgt.bind('<B1-Motion>', manual_conrtol_callback_mouse_motion)
    manual_control_canvas_wgt.bind('<ButtonRelease-1>', manual_control_callback_mouse_released)
    manual_control_canvas_wgt.bind('<Control-1>', manual_control_callback_ctrl_mouse_click)
    manual_control_canvas_wgt.bind('<Shift-1>', manual_control_callback_shift_mouse_click)

# ---------------------------------------------------------------------------

velocity_abs_limit = 100
steering_abs_limit = 100

def ros_controller_set_control(velocity, steering):
    velocity = np.clip(velocity, -velocity_abs_limit, velocity_abs_limit)
    steering = np.clip(steering, -steering_abs_limit, steering_abs_limit)

    cmd = [velocity, steering]

    speed_pub.publish( velocity )
    steer_pub.publish( steering )
    print ('Publishing cmd', cmd)


def ros_controller_init_connection():
    global speed_pub, steer_pub
    
    speed_pub = rospy.Publisher('quadro/speed_perc', Int8, queue_size=1)
    steer_pub = rospy.Publisher('quadro/steer_perc', Int8, queue_size=1)

# ---------------------------------------------------------------------------

if __name__ == '__main__':
    init_gui()
    rospy.init_node('gui_control')
    ros_controller_init_connection()
    root.mainloop()

