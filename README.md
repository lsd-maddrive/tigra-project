# Tigra project

## Подготовка к работе

- Установите требуемые пакеты командой `./scripts/install_packages.sh`
- Установите пакеты для сборки командой `./scripts/install_third_party.sh`
- Соберите требуемые пакеты командой `./scripts/build.sh`
- Установите библиотеки для realsense: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages

### Для разработки

- Установите требуемые пакеты командой `./scripts/install_packages_dev.sh`


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
- [Папка в облаке с корпусами для сенсоров](https://disk.yandex.ru/d/k_3tlJFRWigokQ)

# Заметки

- Firmware перенесен в [другой репо](https://github.com/lsd-maddrive/tigra-firmware). По результатам разработки сделаем объединение, если потребуется.

## Аппаратура

- Камеры
    - [Xiaomi IMILAB](https://market.yandex.ru/product--veb-kamera-xiaomi-imilab-chernyi/668572011?cpa=1&sku=100956420730)
    - [Microsoft Lifecam HD-3000](https://www.microsoft.com/ru-ru/accessories/products/webcams/lifecam-hd-3000)
- [GPS Ublox M8N](https://www.u-blox.com/en/product/neo-m8-series)
    - https://microem.ru/produkti/besprovodnie-tehnologii/glonass-gps-moduli/glonass-gps-modul-neo-m8/
- 

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

## Common

- https://linklab-uva.github.io/autonomousracing/assets/files/L11-compressed.pdf

## GPS

- https://www.github.com/methylDragon/ros-sensor-fusion-tutorial/tree/master/01%20-%20ROS%20and%20Sensor%20Fusion%20Tutorial.md
- https://docs.ros.org/en/noetic/api/robot_localization/html/integrating_gps.html
- http://docs.ros.org/en/noetic/api/robot_localization/html/state_estimation_nodes.html

## Odometry

- https://github.com/ros-controls/ros_controllers/tree/noetic-devel/ackermann_steering_controller
- http://www.lcad.inf.ufes.br/wiki/images/b/b8/Ackerman-steering.pdf
- https://www.theconstructsim.com/wp-content/uploads/2018/05/ros-extra-2.pdf

## Visual odometry

- http://wiki.ros.org/viso2_ros?distro=indigo
- https://github.com/klintan/vo-survey

## Gazebo models

- https://github.com/NevzatBOL/ROS-Beginner/tree/master/catkin_ws/src/sensor_models/urdf

## Movebase

- https://blog.zhaw.ch/icclab/configuring-the-ros-navigation-stack-on-a-new-robot/


# IMU heading (orientation) notes

- the signs of your orientation angles increase in the right direction
- all heading data is assumed to start with its zero point facing east
- If your IMU does not conform to this standard and instead reports zero when facing north, you can still use the yaw_offset parameter to correct this. In this case, the value for yaw_offset would be 𝜋/2 (approximately 1.5707963).
