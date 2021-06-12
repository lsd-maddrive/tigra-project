#!/bin/bash

rostopic pub /quadro/mode_status std_msgs/UInt8 "data: 0" --once

