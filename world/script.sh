#!/bin/bash
 
echo -e "Number of kiwifruit rows to be generated: \c "
read  kiwi

echo -e "Number of picker robots to be generated: \c "
read picker

echo -e "Number of carrier robots to be generated: \c "
read carrier

rosrun stage_ros stageros ./myworld.world