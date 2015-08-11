#!/bin/bash

#move into project directory
cd ugbots_ros

#build before running any robots also run tests
rosmake ugbots_ros --pre-clean -t

#launch the example launch file
#change launch file name to launch file you want to use
roslaunch ugbots_ros example.launch

