#!/bin/bash
source ~/.bashrc

roscd

sleep 5

rosrun stage_ros stageros world/orchard.world
