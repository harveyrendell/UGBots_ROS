#!/bin/bash

gnome-terminal -x bash -c 'roscore'

gnome-terminal -x bash -c 'rosrun stage_ros stageros world/myworld.world'

sleep 3

gnome-terminal -x bash -c 'bash use-launch.bash'
#rosrun stage_ros stageros world/myworld.world
