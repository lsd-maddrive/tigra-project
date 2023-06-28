# Tigra

## Подгрузить все наработки помжно командой:

```bash
roslaunch tigra_software runall.launch
```

## Запуск осуществляется следующей командой в терминале:

```bash
roslaunch tigra_software tigra_software.launch
```

## Настройка планера подргружается с помощью команды: 

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

## Запуск GPS M8N осуществляется следующей командой в терминале: 

```bash
roslaunch tigra_software tigra_gps.launch
```


## Запуск проверки снятия одометрии с GPS и IMU осуществляется следующей командой в терминале: 

```bash
roslaunch tigra_software ekg_odom_test.launch
```

## Notes
Экспорт для окружения для работы симулятора:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find tigra_maps)/models
```