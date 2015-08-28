#!/bin/bash
source ~/.bashrc

roscd

cd ugbots_ros

sleep 5

rosmake ugbots_ros

roslaunch ugbots_ros world.launch
