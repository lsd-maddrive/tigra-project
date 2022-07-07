#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, UInt8
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import time

import numpy as np

MAP_FRAME_ID = 'map'


class GoalDescription:
    def __init__(self, x, y, angle, pose_tol, angle_tol):
        self._x = x
        self._y = y
        self._angle_deg = angle
        self._angle_rad = np.deg2rad(angle)
        self._pose_tol = pose_tol
        self._angle_tol = angle_tol

        self._local_point = np.array([self._x, self._y])

    def eval_tolerance_quat(self, x, y, q):
        euclid_dist = np.sqrt(
            np.sum(
                (np.array([x, y])-self._local_point)**2
            )
        )

        orientation_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        dist_constraint = euclid_dist < self._pose_tol

        if self._angle_tol is not None:
            angle_constraint = np.abs(np.rad2deg(yaw)-self._angle_deg) < self._angle_tol
        else:
            angle_constraint = True

        return dist_constraint and angle_constraint

    def get_move_base_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = MAP_FRAME_ID
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self._x
        goal.target_pose.pose.position.y = self._y

        q = quaternion_from_euler(0, 0, self._angle_rad)

        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        return goal

class GoalsSender:
    def __init__(self, goals):
        self._goals = goals

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.current_goal = None
        self.current_goal_idx = -1
        self._is_sent = False
        self._is_completed = False

    def _next_goal(self):
        self.current_goal_idx += 1
        if self.current_goal_idx >= len(self._goals):
            raise ValueError('Goals completed')

        self.current_goal = self._goals[self.current_goal_idx]

    def step(self):
        if self._is_sent and not self._is_completed:
            return
        else:
            self._is_sent = False
        
        if self.current_goal is None:
            self._next_goal()
        
        if not self._is_sent:
            self._send_goal()
        
    def reset(self):
        self.client.cancel_goal()
    
    def _send_goal(self):
        goal = self.current_goal.get_move_base_goal()
        rospy.loginfo(f'Sending goal {goal}')

        self.client.send_goal(goal,
                    active_cb=self._callback_active,
                    feedback_cb=self._callback_feedback,
                    done_cb=self._callback_done)
        self._is_sent = True
        self._is_completed = False

    def _callback_active(self):
        rospy.loginfo(f"Action server is processing the goal: {self.current_goal}")

    def _callback_done(self, state, result):
        rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))
        if state in [GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED]:
            self._is_completed = True
            self.current_goal = None
        elif state == GoalStatus.ABORTED:
            # Reset
            rospy.loginfo(f'Reset current target: {self.current_goal}')
            self._is_sent = False


    def _callback_feedback(self, feedback):
        # rospy.loginfo("Feedback:%s" % str(feedback))
        fb = feedback

        x = fb.base_position.pose.position.x
        y = fb.base_position.pose.position.y
        q = fb.base_position.pose.orientation

        if self.current_goal.eval_tolerance_quat(x, y, q):
            # Goal reached but only by local tolerance
            self.client.cancel_goal()

            self._is_completed = True
            self.current_goal = None


GOALS_LIST = [
    GoalDescription(x=55, y=0, angle=0, pose_tol=2, angle_tol=None),
    # GoalDescription(x=0, y=0, angle=-25, pose_tol=2, angle_tol=None),     # Debug target
    # GoalDescription(x=60, y=-5, angle=-25, pose_tol=2, angle_tol=None),
    # GoalDescription(x=90, y=-25, angle=-35, pose_tol=2, angle_tol=None),
    # GoalDescription(x=105, y=-48, angle=-45, pose_tol=2, angle_tol=None),
    # GoalDescription(x=90, y=-25, angle=145, pose_tol=2, angle_tol=None),
    # GoalDescription(x=60, y=-5, angle=155, pose_tol=2, angle_tol=None),
    # GoalDescription(x=0, y=5, angle=-180, pose_tol=1, angle_tol=10),
]

if __name__ == '__main__':
    rospy.init_node('control_link')
    print('Ready, go')

    sender = GoalsSender(goals=GOALS_LIST)
    try:
        while not rospy.is_shutdown():
            sender.step()
            time.sleep(1)

    except Exception as e:
        sender.reset()
        print(f'Interrupted: {e}')

    print('Done!')

