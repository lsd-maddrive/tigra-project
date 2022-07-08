https://foxypanda.me/clearing-local-costmap-in-ros-with-pointcloud2/


Infra stereo

https://answers.ros.org/question/398545/intel-realsense-d435i-ros-rtabmap-slam-gets-lost-if-camera-is-moved-slightly-faster/
https://github.com/introlab/rtabmap_ros/blob/master/launch/tests/test_d435i_vio.launch

https://github.com/introlab/rtabmap_ros/issues/565

Квалификация:
- Запустить 2 файла: 
    `roslaunch tigra_software full_move_base_control.launch` 
    `rosrun tigra_software control_link.py`

Заезд:
- Запустить hardware
- Включить управление по джойстику
    `roslaunch tigra_software full_start_joy_mapping.launch`
- Приехать на полигон
- Отключить джойстик и узлы
- Включить ключ
- Включить сигналку
- Зажать кнопку
- Запустить узел move_base
    `roslaunch tigra_software full_move_base_localization.launch`
- Поставить initialpose (rviz)
- Запустить езду
    `rosrun tigra_software control_link.py`
- ПОСЛЕ ОТЖАТИЯ КНОПКИ ПЕРЕЗАПУСТИ КОНТРОЛЛЕР С ЗАЖАТОЙ КНОПКОЙ
- Включить управление по джойстику для отъезда с полигона
    `roslaunch tigra_software full_start_joy_mapping.launch`