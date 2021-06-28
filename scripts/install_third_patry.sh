#!/usr/bin/env bash

THIRD_PARTY_DIR=third_party
mkdir -p $THIRD_PARTY_DIR

# GPS driver

git -C $THIRD_PARTY_DIR clone  https://github.com/KumarRobotics/ublox -b 2.0.0

# ELP stereocamera driver
#   NB - version not fixed as driver is under our development

git -C $THIRD_PARTY_DIR clone https://github.com/lsd-maddrive/elp_stereo_camera
git -C $THIRD_PARTY_DIR/viso2 apply ../patches/viso2.patch

# Libviso2

git -C $THIRD_PARTY_DIR clone https://github.com/srv/viso2 -b melodic_develop
