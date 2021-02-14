
Запуск:

roslaunch tigra_description robot.launch

Управление: 

rostopic pub /tigra/cmd_vel geometry_msgs/Twist -r 5 -- '[10.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
