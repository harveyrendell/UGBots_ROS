#!/bin/bash
source ~/.bashrc

roscd

cd ugbots_ros

sleep 5

rosmake ugbots_ros --pre-clean -t

roslaunch ugbots_ros world.launch
