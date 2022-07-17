# Tigra project

## Содержание репозитория

- [3d_models](3d_models) - 3D модельки корпусов, деталей и т.д.
- [docs](docs) - документация по проекту
    - Инструкции по началу работы (разработке) вы найдете [здесь](docs/DEVELOPMENT.md)
    - А описание робота можно найти [здесь](docs/DESCRIPTION.md)
    - Заметки и полезные ссылки по проекту лежат [здесь](docs/NOTES.md)
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

- Большие ресурсы на [диске](https://disk.yandex.ru/d/1sRPly7asQT_Gg)
- Firmware перенесен в [другой репо](https://github.com/lsd-maddrive/tigra-firmware)
