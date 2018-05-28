# Использование модулей периферии контроллера

## Модуль системы торможения
Driver | Pins | Input / Output
-------|------|-------
PAL  | PA0       | input
ADC2 | PA7 (AN7) | input
GPT7 | Trg       | -

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

## Драйвер управления приводом движения 
Driver | Pins | Input / Output
-------|------|-------
DAC | PA4  | output (channel 1)
PAL | PF12 | output

## Драйвер датчика положения и скорости колес
Driver | Pins | Input / Output
-------|------|-------
EXT1  | PF13 | input
GPT3  | Cntr | -
