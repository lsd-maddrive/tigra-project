# Tigra project

## Пакеты для ROS

- https://github.com/KumarRobotics/ublox

```bash
sudo apt-get install \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-usb_cam \
    ros-$ROS_DISTRO-uvc_camera

```

> Для realsense: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages

# Полезные доки

- [Схема системы](https://drive.google.com/file/d/1iIvuMr4xtmul_ea4DkjYoXMdhi8A7dez/view?usp=sharing)

# Заметки

- Firmware перенесен в [другой репо](https://github.com/lsd-maddrive/tigra-firmware). По результатам разработки сделаем объединение, если потребуется.


# Примеры

## Rosserial socket

- Склонируйте https://github.com/ros-drivers/rosserial
- Соберите `rosserial_client` - `catkin build rosserial_client` в этом пакете
- Сгенерируйте сообщения для встраивания `rosrun rosserial_client make_libraries .`
- Соберите или установети `rosserial_server` (`catkin build rosserial_server`)


# References

## GPS

- https://www.github.com/methylDragon/ros-sensor-fusion-tutorial/tree/master/01%20-%20ROS%20and%20Sensor%20Fusion%20Tutorial.md
- http://docs.ros.org/en/lunar/api/robot_localization/html/integrating_gps.html
- http://docs.ros.org/en/kinetic/api/robot_localization/html/state_estimation_nodes.html

