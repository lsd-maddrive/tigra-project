# Tigra

## Установка:

Клонируем репозиторий, скачивая ветку `feature/gazebo_model`: 

```bash
git clone --branch feature/gazebo_model https://github.com/lsd-maddrive/tigra-project.git
```
После собираем пакет с помощью команды `catkin_make` в Вашем ws.

## Запуск осуществляется следующей командой в терминале:

```bash
roslaunch tigra_description tigra.launch
```

> В лаунч-файле Вы можете поставить модель без пружин, поменяв имя модели с `tigra.urdf.xacro` на `tigra_no_spring.urdf.xacro`.

## Управление задается : 

```bash
rostopic pub /tigra/cmd_vel geometry_msgs/Twist -r 5 -- '[3.0, 0.0, 0.0]' '[0.0, 0.0, 2.0]'
```
>  Разберем эту команду по частям:

- rostopic pub — публикуем сообщение в данной теме.
- -1 — публиковать только одно сообщение, затем выйти.
- -r 5 — публиковать сообщения с частотой 5 Гц.
- /tigra/cmd_vel — имя темы, в которую публиковать.
- geometry_msgs/Twist — тип сообщения, который использовался при публикации темы.
- — — двойное тире сообщает анализатору синтаксиса, что ни один из следующих аргументов не является параметром. Это необходимо в тех случаях, когда в аргумент входит дефис, например в отрицательных числах.
- '[3.0, 0.0, 0.0]' '[0.0, 0.0, 2.0]' — сообщение geometry_msgs/Twist имеет два вектора: `linear` и `angular`, каждый из которых состоит из трех числовых элементов c плавающей точкой. В нашем случае, '[3.0, 0.0, 0.0]' становится линейным значением с x=3, y=0, z=0 и '[0.0, 0.0, 2.0]' является угловым значением с x=0, y=0, z=2. Эти аргументы имеют YAML- синтаксис.

#### Примечание: Нельзя установить одновременно -r и -1

# Описание параметров модели и частей

1. В коде имеется нулевая инерция. Она была добавлена, чтобы реализовать идею о том, как соединить [джоинты](http://wiki.ros.org/urdf/XML/joint) последовательно, не имея лишних [линков](http://wiki.ros.org/urdf/XML/link). Конечно, линки присутствуют, но они не имеют ни тела, ни визуализации, имеют лишь момент инерции.

#### Для будущих проектов(!) Если у Вас возникнут проблемы с соединениямми на моделях с большой массой, то попробуйте увеличить массу и моменты инерций (ixx, iyy, izz) на нулевой инерции.

2. Для реализации пружин был использован joint типа `prismatic`. Необходимо настроить его на смещение по оси z, задав `limits` на минимум и максимум по оси. 

3. Дополняя предыдуший пункт о пружинах, можно сказать про его настройку. В коде присутствует такие строчки: 

```bash
   <gazebo reference="${name}_spring_joint">
    	<implicitSpringDamper>true</implicitSpringDamper>
    	<springStiffness>${springStiffness}</springStiffness>
    	<springReference>${springReference}</springReference>
    </gazebo>
```
Значение `springStiffness` отвечает за жесткость пружины, а `springReference` за ...

## Полезные ссылки:

- [ROS cource](https://github.com/KaiL4eK/ros_course)
- [zaWRka description](https://github.com/lsd-maddrive/zaWRka-project/tree/develop/wr8_description)
- [Answers and questions Gazebosim](https://answers.gazebosim.org/questions/)
- [YouTube cource about ROS and Gazebo (The Construct)](https://www.youtube.com/channel/UCt6Lag-vv25fTX3e11mVY1Q)
- [DEM карта в недалеком будущем](http://gazebosim.org/tutorials/?tut=dem)
- [Database models](https://github.com/osrf/gazebo_models)


# Проверка корректности модели

`check_urdf <(xacro tigra_description/urdf/tigra.urdf.xacro)`
