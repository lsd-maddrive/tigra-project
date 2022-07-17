# Заметки и полезные ссылки

- [Заметки](#заметки)
  - [IMU heading (orientation) notes](#imu-heading-orientation-notes)
- [Ссылки](#ссылки)
  - [Common](#common)
  - [GPS](#gps)
  - [Odometry](#odometry)
  - [Visual odometry](#visual-odometry)
  - [Gazebo models](#gazebo-models)
  - [Movebase](#movebase)
  - [Planning](#planning)
- [Список ссылок по системе визуального восприятия](#список-ссылок-по-системе-визуального-восприятия)

## Заметки

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

### IMU heading (orientation) notes

- the signs of your orientation angles increase in the right direction
- all heading data is assumed to start with its zero point facing east
- If your IMU does not conform to this standard and instead reports zero when facing north, you can still use the yaw_offset parameter to correct this. In this case, the value for yaw_offset would be 𝜋/2 (approximately 1.5707963).


## Ссылки

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


### Planning

Планирование движения теория методы управления робототехническими приложениями ИТМО 2016г (Глава 2):
https://books.ifmo.ru/file/pdf/2094.pdf

Решение задачи для беспилотных автомобилей (англ. Self-driving cars):
https://neerc.ifmo.ru/wiki/index.php?title=Задача_планирования_движения

Алгоритм A*:
https://neerc.ifmo.ru/wiki/index.php?title=Алгоритм_A*

Алгоритмы построения пути для беспилотного автомобиля. Лекция Яндекса:
https://habr.com/ru/company/yandex/blog/340674/

МАТЕМАТИЧЕСКОЕ И ПРОГРАММНОЕ ОБЕСПЕЧЕНИЕ ВЫЧИСЛИТЕЛЬНЫХ МАШИН, КОМПЛЕКСОВ И КОМПЬЮТЕРНЫХ СЕТЕЙ (ТЕХНИЧЕСКИЕ НАУКИ) стастья из журнала:
https://hi-tech.asu.edu.ru/files/3(47)/70-82.pdf

РАЗРАБОТКА И ИМПЛЕМЕНТАЦИЯ СПЛАЙН-АЛГОРИТМА ПЛАНИРОВАНИЯ ПУТИ В СРЕДЕ ROS/GAZEBO:
http://proceedings.spiiras.nw.ru/index.php/sp/article/view/4033/2517


## Список ссылок по системе визуального восприятия

- Книги по ROS на русском
```
https://habr.com/ru/post/663230/ 
```
- Одометрия. Базовые понятия.
```
https://robocraft.ru/blog/technology/736.html
```
- Визуальная одометрия. База.
```
https://robocraft.ru/blog/computervision/738.html
```

- Методы оценки положения объекта в пространстве. Журнал 2013 год. Есть сравнение методов.
```
https://docplayer.com/47755933-Metody-ocenki-polozheniya-obekta-v-prostranstve.html
```

- Обзор методов визуальной одометрии в ROS: использование камер глубины. Пост на хабре. Нескольких алгоритмов визуальной одометрии на ROS'е
```
https://habr.com/ru/post/404757/
```
- Запуск Intel RealSense d435i в rtabmap
```
http://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping
```


- Обзор современных методов визуальной одометрии. Статья из журнала 2019 года. Описываются задачи систем визуальной одометрии и SLAM и их основные применения; способы создания систем.
```
http://cte.eltech.ru/ojs/index.php/kio/article/view/1596/1582
https://github.com/MikhailTerekhov/mdso
```

- Визуальный одометр. Вырезка из журнала 2012 год. Описан анализ последовательности изоображений. Много математики.
```
http://engjournal.ru/articles/249/249.pdf
```

- Обзор и алгоритм с пояснениями. Вики. Есть что потыкать.
```
https://en.wikipedia.org/wiki/Visual_odometry
```

- Визуальная одометрия с длительным прослеживанием особенностей. Журнал 2017 год. Алгоритм "длительного прослеживания"; Стоит взглянуть.
```
http://ics.khstu.ru/media/2017/N53_02.pdf
```

- Визуальная одометрия в методах машинного контроля. ВКР профессор Терехов. Несколько алгоритмов и RANSAC.
```
https://oops.math.spbu.ru/SE/diploma/2017/pi/Rabochy.pdf
```

- Алгоритм построения трехмерной карты окружающей среды с использованием камеры глубины.
```
http://www.jip.ru/2019/355-365-2019.pdf
```

Гироскоп
```
https://habr.com/ru/post/118192/
```

