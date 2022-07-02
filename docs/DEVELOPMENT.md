# Инструкции по разработке

## Требования

- Ubuntu 20.04
- Установленный [ROS Noetic](http://wiki.ros.org/noetic/Installation)
- Готовность разбираться и улучшать систему! =)
- Установленный плагин в VSCode для чтения drawio диаграмм (или используйте https://app.diagrams.net/)

## Некоторые правила разработки

- Перед началом работы над задачей (созданием ветки под задачу) сделайте `git pull` в `develop` ветке
- Каждый комит должен содержать номер таски, в рамках которой делалась работа в этом комите. Пример: `#88 designed config structure and added comments`
- Имя ветки, в которой ведется работа, должна содержать номер таски. Пример: `feature/88_radar_config`
- При создании задачи указывайте проект в меню Projects
- После завершения работы над задачей создавайте Pull Request на вивание ветки в `develop`. При создании указывайте ревьюверов (как минимум ведущего, можно и остальных), проект в Projects и связанные задачи (Linked Issues)
- После апрува сливает в `develop` ведущий разработчик


## Подготовка к работе

Для работы в среде ROS требуется установить в workspace [репозиторий с общими тулами для ROS](https://github.com/lsd-maddrive/maddrive_ros_shared).

### Установка catkin_tools

- Установите с помощью `sudo apt install python3-catkin-tools`

или

- Создайте виртуальное окружение (`python3.8 -m venv venv38`) и в него поставьте бинарник с помощью `pip install -r requirements.dev.txt`

### Преднастройка

- В пакете `maddrive_ros_shared` установите пакеты для сборки командой `./scripts/install_third_party.sh`
- Установите требуемые пакеты командой `./scripts/install_packages.sh`
- Соберите требуемые пакеты командой `./scripts/build.sh`

### Настройка камеры RealSense

- Установите библиотеки для realsense: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages
> https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md

### Настройки пользователей для работы с камерой и serial

- `sudo usermod -aG dialout $USER` - для работы с Serial
- `sudo usermod -aG video $USER`


## Запуск на роботе

### Подготовка

- Поставьте `catkin_tools` в виртуальное окружение или в систему
- Соберите пакеты командой
```bash
catkin build \
    tigra_msgs \
    tigra_description \
    tigra_software \
    tigra_vision \
    tigra_maps
```

### Server-side для связи с МК

- Настройте на компе статический адрес `192.168.10.1` (на МК установлен `192.168.10.2` с gateway в адрес компа)
- Установите `rosserial_client` и `rosserial_server`
- Запустите `rosserial_server` - это поднимет сервер на порту 23456
```bash
    roslaunch tigra_software uc_socket_server.launch
```
> Должны опубликоваться топики
```
/tigra/state
/tigra/state_cmd
```

- Запустите `uc_convertion_layer.launch` - это запускает преобразования для привычного для ROS формата
```bash
    roslaunch tigra_software uc_convertion_layer.launch
```
> Должны опубликоваться топики
```
/tigra/cmd_vel
/tigra/state
/tigra/state_cmd
/tigra/wheel_odom
```

### Телеуправление

> На этом связь с роботом должна быть налажена и опубликованы топики:
```
/tigra/cmd_vel
/tigra/state
/tigra/state_cmd
/tigra/wheel_odom
```

- Для управления с клавиатуры необходимо запустить лаунч `start_keyboard_teleop.launch` и управлять в терминале. Настраиваются пределы в самом лаунче.
```bash
    roslaunch tigra_software start_keyboard_teleop.launch
```

- Для управления с джойстика необходимо запустить лаунч `start_joy_teleop.launch`. Настраиваются пределы в самом лаунче.
```bash
    roslaunch tigra_software start_joy_teleop.launch
```

> Правый стик - скорость (верх-низ), левый стик - поворот (лево-право)

> Джойстик должен быть в режиме "D"

### Тест камеры

- Для запуска различных тестов запустить неободимый лаунч файл. Названия файлов сформированы по принципу `test_(объект тестирования)_(тестируемая функция).launch`.

> Например, для вывода изображения камеры `Intel (D435)` использовать лаунч `roslaunch tigra_vision test_d435_view.launch`. 

### Калибровка камеры

- Для начала калибровки запустить лаунч `calibration.launch`.
- Инструкция по калибровке камеры и параметры `cameracalibrator.py` можно посмотреть тут:
> http://wiki.ros.org/camera_calibration
- После калибровки не забыть нажать кнопку `Save` и переметстить калибровочные файлы из `/tmp/calibrationdata.tar.gz` в папку `tigra-project/tigra_vision/calib_info/(имя камеры)`.
- Проверить результат калировки можно с помощью `check_calib.launch` или тестов для камеры.

- Если после запуска камеры изображение наклонено (в RVIZ хорошо видно, когда видимый камерой пол под углом к горизонтальной сетке), необходимо поменять значения углов (4,5 или 6 аргумент) для объекта `tf_front_axis_2_rs_camera` в `tigra_software/launch/drivers/tigra_tf_transforms.launch`.

### Все и разом

- Скрипт запуска всех нужных скриптов микроконтроллера и управление джойстиком - `full_start_joy_control.launch`


### Сборка `ros_lib` для встраивания в микроконтроллерную связку

- Установите `rosserial_client`
- соберите наши пакеты `tigra_software` и `tigra_msgs` (`catkin build tigra_software tigra_msgs`)
- Сгенерируйте сообщения для встраивания `rosrun tigra_msgs create_uc_ros_lib.py`
- Заберите из папки пакета `tigra_msgs` папку `ros_lib` и закиньте себе в сборку для МК

## Использование телефона в качестве IMU/GPS источника

- Ставим апу https://play.google.com/store/apps/details?id=tech.unismart.dc&hl=en_US&gl=US
- Подсоединяем телефон по USB к компу и включаем модем
- Смотрим IP компа, создаем сервер и задачу в приложении
    - Адрес сервера: ws://192.168.1.45:5000//sensors
    - Протокол: ws
- Настраиваем задачу
    - Датчики: acceleromenter, gyroscope, rotation, location
    - Период: 20
- Заходим в настройки приложения, выбираем формат данных - строгий JSON с названиями датчиков
- Пускаем
```bash
roslaunch tigra_software phone_server.launch
```
- Должны опубликоваться топики `/tigra/phone_sensors/imu` и `/tigra/phone_sensors/gps`

## Старт симулятора

`roslaunch tigra_software start_sim_robot.launch`


> Для запуска на других картах настройте переменную окружения `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find tigra_maps)/models`

