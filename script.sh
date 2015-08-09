#!/bin/bash
 
echo "Number of picker robots to be generated: \c "
read picker

echo "Number of carrier robots to be generated: \c "
read carrier

rm ugbots_ros/launch/example2.launch

echo  \<launch\> > ugbots_ros/launch/example2.launch

i=0
j=0

while [ $i -lt $picker ];
do

echo \<group ns=\"robot_$i\"\> >> ugbots_ros\/launch\/example2.launch
echo \<node pkg=\"ugbots_ros\" name=\"robotnode\" type=\"PB\"\/\> >> ugbots_ros\/launch\/example2.launch 
echo \<\/group\> >> ugbots_ros\/launch\/example2.launch

i=$(($i+1))

done

while [ $j -lt $carrier ];
do

echo \<group ns=\"robot_$i\"\> >> ugbots_ros\/launch\/example2.launch
echo \<node pkg=\"ugbots_ros\" name=\"robotnode\" type=\"CB\"\/\> >> ugbots_ros\/launch\/example2.launch
echo \<\/group\> >> ugbots_ros\/launch\/example2.launch

j=$(($j+1))
i=$(($i+1))

done

echo  \<launch\> >> ugbots_ros/launch/example2.launch

rosrun stage_ros stageros ./world/myworld.world
