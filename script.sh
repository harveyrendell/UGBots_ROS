#!/bin/bash
 
echo "Number of picker robots to be generated: \c "
read picker

echo "Number of carrier robots to be generated: \c "
read carrier

echo "Number of workers to be generated: \c "
read worker

echo "Number of animals to be generated: \c "
read animal

rm ugbots_ros/launch/example2.launch
rm world/robotinstances.inc
rm world/workerinstances.inc
rm world/animalinstances.inc

echo  \<launch\> > ugbots_ros/launch/example2.launch
echo include \"robots.inc\" > world/robotinstances.inc
echo include \"workers.inc\" > world/workerinstances.inc
echo include \"animals.inc\" > world/animalinstances.inc

i=0
j=0
w=0
a=0

while [ $i -lt $picker ];
do

echo \<group ns=\"robot_$i\"\> >> ugbots_ros/launch/example2.launch
echo \<node pkg=\"ugbots_ros\" name=\"robotnode\" type=\"PB\"\/\> >> ugbots_ros/launch/example2.launch 
echo \<\/group\> >> ugbots_ros\/launch\/example2.launch

echo pickerRobot\(pose [ 0 $((33-$(($i * 5)))) 0 0 ]\ name \"R$i\" color \"red\"\) >> world/robotinstances.inc

i=$(($i+1))

done

while [ $j -lt $carrier ];
do

echo \<group ns=\"robot_$i\"\> >> ugbots_ros\/launch\/example2.launch
echo \<node pkg=\"ugbots_ros\" name=\"robotnode\" type=\"CB\"\/\> >> ugbots_ros\/launch\/example2.launch
echo \<\/group\> >> ugbots_ros\/launch\/example2.launch

echo carrierRobot\(pose [ 0 $((-33+$(($j*5)))) 0 0 ] name \"R$i\" color \"blue\"\) >> world/robotinstances.inc

j=$(($j+1))
i=$(($i+1))

done

while [ $w -lt $worker ];
do

echo \<group ns=\"robot_$i\"\> >> ugbots_ros/launch/example2.launch
echo \<node pkg=\"ugbots_ros\" name=\"workernode\" type=\"Person\"\/\> >> ugbots_ros/launch/example2.launch 
echo \<\/group\> >> ugbots_ros/launch/example2.launch

echo worker\(pose [ 0 $((1+$(($w * 2)))) 0 0 ] name \"W$w\" color \"black\" \) >> world/workerinstances.inc
w=$(($w+1))
i=$(($i+1))

done

while [ $a -lt $animal ];
do

echo \<group ns=\"robot_$i\"\> >> ugbots_ros/launch/example2.launch
echo \<node pkg=\"ugbots_ros\" name=\"workernode\" type=\"Animal\"\/\> >> ugbots_ros/launch/example2.launch 
echo \<\/group\> >> ugbots_ros/launch/example2.launch

echo animal\( pose [ 0 $((-1-$(($a * 2)))) 0 0 ] name \"A$a\" color \"brown\" \) >> world/animalinstances.inc
a=$(($a+1))
i=$(($i+1))

done

echo  \<launch\> >> ugbots_ros/launch/example2.launch

rosrun stage_ros stageros ./world/myworld.world
