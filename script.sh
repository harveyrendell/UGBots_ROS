#!/bin/bash
 
echo "Number of picker robots to be generated: \c "
read picker

echo "Number of carrier robots to be generated: \c "
read carrier

rm ugbots_ros/launch/example2.launch
rm world/robotinstances.inc

echo  \<launch\> > ugbots_ros/launch/example2.launch
echo include \"robots.inc\" > world/robotinstances.inc

i=0
j=0

while [ $i -lt $picker ];
do

echo \<group ns=\"robot_$i\"\> >> ugbots_ros\/launch\/example2.launch
echo \<node pkg=\"ugbots_ros\" name=\"robotnode\" type=\"PICKER\"\/\> >> ugbots_ros\/launch\/example2.launch 
echo \<\/group\> >> ugbots_ros\/launch\/example2.launch

echo pickerRobot\(pose [ 0 $((33-$(($i * 5)))) 0 0 ]\ name \"R$i\" color \"red\"\) >> world\/robotinstances.inc

i=$(($i+1))

done

while [ $j -lt $carrier ];
do

echo \<group ns=\"robot_$i\"\> >> ugbots_ros\/launch\/example2.launch
echo \<node pkg=\"ugbots_ros\" name=\"robotnode\" type=\"CARRIER\"\/\> >> ugbots_ros\/launch\/example2.launch
echo \<\/group\> >> ugbots_ros\/launch\/example2.launch

echo carrierRobot\(pose [ 0 $((-33+$(($j*5)))) 0 0 ]\ name \"R$i\" color \"blue\"\) >> world\/robotinstances.inc

j=$(($j+1))
i=$(($i+1))

done

echo  \<\/launch\> >> ugbots_ros/launch/example2.launch

rosrun stage_ros stageros ./world/myworld.world
