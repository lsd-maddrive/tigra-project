# Использование модулей периферии контроллера

## Модуль системы торможения
Driver | Pins | Input / Output
-------|------|-------
PAL  | PA0 | input
ADC1 | PA3 | input (AN3)
GPT6 | Trg | -

## Модуль ручки тормоза
Driver | Pins | Input / Output
-------|------|-------
EXT1 | PA1 | input

## Драйвер управления приводом руля 
Driver | Pins | Input / Output
-------|------|-------
PWM1 | PE9 | output (channel 0)
PAL  | PE3 | output

## Драйвер управления приводом тормоза 
Driver | Pins | Input / Output
-------|------|-------
PWM1 | PE11 | output (channel 1)
PAL  | PE15 | output
PAL  | PG1  | output

## Драйвер управления приводом движения 
Driver | Pins | Input / Output
-------|------|-------
DAC | PA4  | output (channel 1)
PAL | PD3 | output (open-drain)

## Драйвер датчика положения и скорости колес
Driver | Pins | Input / Output
-------|------|-------
EXT1  | PF13 | input
GPT3  | Cntr | -

## Драйвер датчика положения руля и датчика усилия привода руля
Driver | Pins | Input / Output
-------|------|-------
ADC1 | PC0  | input (AN 10)
ADC1 | PC3  | input (AN 13)
GPT6 | Trg  | -

## Драйвер системы связи ROS
Driver | Pins | Input / Output
-------|------|-------
SD5 | PC12 | TX
+++ | PD2  | RX 

## Драйвер УЗ-датчиков
Driver | Pins | Input / Output
-------|------|-------
GPT4 | Trg  | -
PAL | PG2  | output
PAL | PG3  | output
SD5 | PD2  | RX
SD4 | PC11 | RX 

## Драйвер ИК-датчика 
Driver | Pins | Input / Output
-------|------|-------
GPT8 | Trg  | -
ADC3 | PF3  | input (AN 9)

## Модули света
Driver | Pins | Input / Output
-------|------|-------
PAL | E2  | output (open-drain)
PAL | E4 | output (open-drain)
PAL | E6 | output (open-drain)
PAL | E3 | output (open-drain)
