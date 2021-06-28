#!/usr/bin/env bash

PKG_PATH=$(rospack find tigra_vision) 

sudo cp ${PKG_PATH}/udev/10.depnb.rules /etc/udev/rules.d/
