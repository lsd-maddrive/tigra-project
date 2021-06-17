# From: http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html
# Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
# Speed of wheel rotation [rad/s]
float32 rotation_speed
# Steering angle [rad]
float32 angle_steering
