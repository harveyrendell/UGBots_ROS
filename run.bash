#!/bin/bash

#Open terminal for to run roscore
gnome-terminal -x bash -c 'roscore'

#wait 3 seconds for roscore to initialise
sleep 3

#Open terminal to run stageros for world
gnome-terminal -x bash -c 'rosrun stage_ros stageros world/myworld.world'

#FOR LAUNCH FILE
#Open terminal to run script that runs the example launch file
gnome-terminal -x bash -c 'bash use-launch.bash'

#FOR INDIVIDUAL ROSRUN
#Open terminal to run script that runs an individual node
#REMEMBER to edit the use-node.bash script to run the node you want to run
#gnome-terminal -x bash -c 'bash use-node.bash'
