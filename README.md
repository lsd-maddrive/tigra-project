# Tigra project

## Пакеты для ROS

- https://github.com/KumarRobotics/ublox
- https://github.com/srv/viso2/tree/melodic_develop

> Для сборки на `noetic` достаточно из CMakeLists.txt удалить Boost зависимость - signals

```bash
sudo apt-get install \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-rosserial-server \
    ros-$ROS_DISTRO-rosserial-client \
    ros-$ROS_DISTRO-rtabmap-ros \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-move-base \
    ros-$ROS_DISTRO-global-planner \
    ros-$ROS_DISTRO-teb-local-planner
```

### Для симулятора

```bash
sudo apt install \
    ros-$ROS_DISTRO-hector-gazebo-plugins
```

> Для realsense: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages

# Старт симулятора

`roslaunch tigra_software start_sim_robot.launch`

# Некоторые правила разработки

- Перед началом работы над задачей (созданием ветки под задачу) сделайте `git pull` в `develop` ветке
- Каждый комит должен содержать номер таски, в рамках которой делалась работа в этом комите. Пример: `#88 designed config structure and added comments`
- Имя ветки, в которой ведется работа, должна содержать номер таски. Пример: `feature/88_radar_config`
- При создании задачи указывайте проект в меню Projects
- После завершения работы над задачей создавайте Pull Request на вивание ветки в `develop`. При создании указывайте ревьюверов (как минимум ведущего, можно и остальных), проект в Projects и связанные задачи (Linked Issues)
- После апрува сливает в `develop` ведущий

# Полезные доки

- [Схема системы](https://drive.google.com/file/d/1iIvuMr4xtmul_ea4DkjYoXMdhi8A7dez/view?usp=sharing)

# Заметки

- Firmware перенесен в [другой репо](https://github.com/lsd-maddrive/tigra-firmware). По результатам разработки сделаем объединение, если потребуется.


# Примеры

## Rosserial socket

- Соберите пример с помощью скрипта [build.sh](samples/rosserial_socket_sample/build.sh)
- Установите `rosserial_client` и `rosserial_server` (команда есть выше)
- Запустите `rosserial_server` - это поднимет сервер на порту 23456 (`roslaunch tigra_software uc_socket_server.launch`)
- Запустите пример из папки `build` в примере и можете общаться через `rostopic`

# Сборка `ros_lib` для встраивания в микроконтроллерную связку

- Установите `rosserial_client`
- Установите `rosserial_server`
- соберите наши пакеты `tigra_software` и `tigra_msgs` (`catkin build tigra_software tigra_msgs`)
- Сгенерируйте сообщения для встраивания `rosrun tigra_msgs create_uc_ros_lib.py`
- Заберите из папки пакета `tigra_msgs` папку `ros_lib` и закиньте себе в сборку для МК

# References

## GPS

- https://www.github.com/methylDragon/ros-sensor-fusion-tutorial/tree/master/01%20-%20ROS%20and%20Sensor%20Fusion%20Tutorial.md
- http://docs.ros.org/en/lunar/api/robot_localization/html/integrating_gps.html
- http://docs.ros.org/en/kinetic/api/robot_localization/html/state_estimation_nodes.html
- https://github.com/KumarRobotics/ublox

## Odometry

- https://github.com/ros-controls/ros_controllers/tree/noetic-devel/ackermann_steering_controller
- http://www.lcad.inf.ufes.br/wiki/images/b/b8/Ackerman-steering.pdf

## Visual odometry

- http://wiki.ros.org/viso2_ros?distro=indigo
- https://github.com/klintan/vo-survey

## Gazebo models

- https://github.com/NevzatBOL/ROS-Beginner/tree/master/catkin_ws/src/sensor_models/urdf

## Movebase

- https://blog.zhaw.ch/icclab/configuring-the-ros-navigation-stack-on-a-new-robot/
