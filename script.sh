#!/bin/bash
 
echo "Number of picker robots to be generated: \c "
read picker

echo "Number of carrier robots to be generated: \c "
read carrier

echo "Number of workers to be generated: \c "
read worker

echo "Number of animals to be generated: \c "
read animal

mkdir -p world/config

rm ugbots_ros/launch/world.launch
rm world/config/robotinstances.inc
rm world/config/workerinstances.inc
rm world/config/animalinstances.inc

echo  \<launch\> > ugbots_ros/launch/world.launch
echo include \"robots.inc\" > world/config/robotinstances.inc
echo include \"workers.inc\" > world/config/workerinstances.inc
echo include \"animals.inc\" > world/config/animalinstances.inc

i=0
j=0
w=0
a=0

while [ $i -lt $picker ];
do

echo \<group ns=\"robot_$i\"\> >> ugbots_ros\/launch\/world.launch
echo \<node pkg=\"ugbots_ros\" name=\"robotnode\" type=\"PICKER\"\/\> >> ugbots_ros\/launch\/world.launch 
echo \<\/group\> >> ugbots_ros\/launch\/world.launch

echo pickerRobot\(pose [ 0 $((33-$(($i * 5)))) 0 0 ]\ name \"R$i\" color \"red\"\) >> world/config/robotinstances.inc

i=$(($i+1))

done

while [ $j -lt $carrier ];
do

echo \<group ns=\"robot_$i\"\> >> ugbots_ros\/launch\/world.launch
echo \<node pkg=\"ugbots_ros\" name=\"robotnode\" type=\"CARRIER\"\/\> >> ugbots_ros\/launch\/world.launch
echo \<\/group\> >> ugbots_ros\/launch\/world.launch

echo carrierRobot\(pose [ 0 $((-33+$(($j*5)))) 0 0 ] name \"R$i\" color \"blue\"\) >> world/config/robotinstances.inc

j=$(($j+1))
i=$(($i+1))

done

while [ $w -lt $worker ];
do

echo \<group ns=\"robot_$i\"\> >> ugbots_ros/launch/world.launch
echo \<node pkg=\"ugbots_ros\" name=\"workernode\" type=\"WORKER\"\/\> >> ugbots_ros/launch/world.launch 
echo \<\/group\> >> ugbots_ros/launch/world.launch

echo worker\(pose [ 0 $((1+$(($w * 2)))) 0 0 ] name \"W$w\" color \"black\" \) >> world/config/workerinstances.inc
w=$(($w+1))
i=$(($i+1))

done

while [ $a -lt $animal ];
do

echo \<group ns=\"robot_$i\"\> >> ugbots_ros/launch/world.launch
echo \<node pkg=\"ugbots_ros\" name=\"workernode\" type=\"DOG\"\/\> >> ugbots_ros/launch/world.launch 
echo \<\/group\> >> ugbots_ros/launch/world.launch

echo animal\( pose [ 0 $((-1-$(($a * 2)))) 0 0 ] name \"A$a\" color \"brown\" \) >> world/config/animalinstances.inc
a=$(($a+1))
i=$(($i+1))

done

echo  \<\/launch\> >> ugbots_ros/launch/world.launch
