# Tigra project

## Содержание репозитория

- [3d_models](3d_models) - 3D модельки корпусов, деталей и т.д.
- [docs](docs) - документация по проекту
    - Инструкции по началу работы (разработке) вы найдете [здесь](docs/DEVELOPMENT.md)
    - А описание робота можно найти [здесь](docs/DESCRIPTION.md)
- [scripts](scripts) - скрипты для настройки/запуска чего либо
- [ackermann_raw_controller_plugin](ackermann_raw_controller_plugin) - плагин для модели в симуляторе, который имеет интерфейс, схожий с реальным роботом
    > Сам плагин передает сырые состояния, которые обрабатываются узлом `tigra_software/robot_layer`
- [third_party](third_party) - патчи для сторонних пакетов и место для их загрузки
- [tigra_description](tigra_description) - описание робота (конфиги, модельки и т.д.)
- [tigra_maps](tigra_maps) - карты для симулятора
- [tigra_msgs](tigra_msgs) - прототипы используемых сообщений
- [tigra_software](tigra_software) - основной стек скриптов запуска для реального робота и симулятора
- [tigra_vision](tigra_vision) - пакет по визуальной части робота (работа с камерами, визуальная одометрия и т.д.)

## Дополнительные ресурсы

- [Описание робота](https://lavish-podium-945.notion.site/Tigra-c740280f21394deb8394ce08008f9b60)
- Большие ресурсы на [диске](https://disk.yandex.ru/d/1sRPly7asQT_Gg)
- [Схемы системы](docs/Schemes.drawio) (для чтения установите drawio расширение в VSCode или используйте [diagrams.net](https://app.diagrams.net/))
- Firmware перенесен в [другой репо](https://github.com/lsd-maddrive/tigra-firmware).


### Аппаратура

- Камеры
    - [Xiaomi IMILAB](https://market.yandex.ru/product--veb-kamera-xiaomi-imilab-chernyi/668572011?cpa=1&sku=100956420730)
    - [Microsoft Lifecam HD-3000](https://www.microsoft.com/ru-ru/accessories/products/webcams/lifecam-hd-3000)
    - [GPS Ublox M8N](https://www.u-blox.com/en/product/neo-m8-series)
        - https://microem.ru/produkti/besprovodnie-tehnologii/glonass-gps-moduli/glonass-gps-modul-neo-m8/
    - [Cтереокамеры ELP](http://www.elpcctv.com/elp-720p-cmos-ov9712-sensor-mjpeg-yuy2-dual-lens-stereo-usb-camera-module-with-uvc-for-robot-vision-p-135.html)

### Common

- https://linklab-uva.github.io/autonomousracing/assets/files/L11-compressed.pdf
- http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot
- http://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping

### GPS

- https://www.github.com/methylDragon/ros-sensor-fusion-tutorial/tree/master/01%20-%20ROS%20and%20Sensor%20Fusion%20Tutorial.md
- https://docs.ros.org/en/noetic/api/robot_localization/html/integrating_gps.html
- http://docs.ros.org/en/noetic/api/robot_localization/html/state_estimation_nodes.html

### Odometry

- https://github.com/ros-controls/ros_controllers/tree/noetic-devel/ackermann_steering_controller
- http://www.lcad.inf.ufes.br/wiki/images/b/b8/Ackerman-steering.pdf
- https://www.theconstructsim.com/wp-content/uploads/2018/05/ros-extra-2.pdf
- https://docs.swiftnav.com/wiki/ROS_Integration_Guide
- http://wiki.ros.org/rtabmap_ros/Tutorials/StereoOutdoorMapping

### Visual odometry

- http://wiki.ros.org/viso2_ros?distro=indigo
- https://github.com/klintan/vo-survey

### Gazebo models

- https://github.com/NevzatBOL/ROS-Beginner/tree/master/catkin_ws/src/sensor_models/urdf

### Movebase

- https://blog.zhaw.ch/icclab/configuring-the-ros-navigation-stack-on-a-new-robot/


## IMU heading (orientation) notes

- the signs of your orientation angles increase in the right direction
- all heading data is assumed to start with its zero point facing east
- If your IMU does not conform to this standard and instead reports zero when facing north, you can still use the yaw_offset parameter to correct this. In this case, the value for yaw_offset would be 𝜋/2 (approximately 1.5707963).


## Notes

- Как включить уровень отладки

```xml
    <node pkg="rosservice" type="rosservice" name="set_log_level_1" args="call --wait /tigra/rosserial_server/set_logger_level 'ros.rosserial_server' 'debug'" />
    <node pkg="rosservice" type="rosservice" name="set_log_level_2" args="call --wait /tigra/rosserial_server/set_logger_level 'ros.roscpp' 'debug'" />
```

- Дополнительные экспорты для окружения

```bash
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.8/site-packages
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libjemalloc.so.2 # https://github.com/SteveMacenski/spatio_temporal_voxel_layer/issues/167
```

