#!/bin/bash
source ~/.bashrc

roscd

cd ugbots_ros


rosmake ugbots_ros

roslaunch ugbots_ros world.launch
