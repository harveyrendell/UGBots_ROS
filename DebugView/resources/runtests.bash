#!/bin/bash
source ~/.bashrc
roscd

cd ugbots_ros

xterm -hold -e make test
