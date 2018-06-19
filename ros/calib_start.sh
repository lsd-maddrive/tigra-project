#!/bin/bash

rosrun camera_calibration cameracalibrator.py \
	--size 9x7 --square 0.023 \
	right:=/stereo/right/image_raw left:=/stereo/left/image_raw \
	right_camera:=/stereo/right left_camera:=/stereo/left  \
	--no-service-check --approximate=0.01

