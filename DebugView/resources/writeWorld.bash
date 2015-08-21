#!/bin/bash

source ~/.bashrc
roscd

picker=$1
carrier=$2
worker=$3
visitor=$4
dog=$5
possum=$6
tractor=$7

i=0
j=0
w=0
a=0
v=0
d=0
c=0
po=0
t=0

mkdir -p world/

rm ugbots_ros/launch/world.launch
rm world/config/robotinstances.inc
rm world/config/workerinstances.inc
rm world/config/animalinstances.inc
rm world/config/visitorinstances.inc

echo  \<launch\> > ugbots_ros/launch/world.launch
echo include \"robots.inc\" > world/config/robotinstances.inc
echo include \"workers.inc\" > world/config/workerinstances.inc
echo include \"dogs.inc\" > world/config/animalinstances.inc
#echo include \"possums.inc\" >> world/config/animalinstances.inc
echo include \"workers.inc\" >> world/config/visitorinstances.inc



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

while [ $v -lt $visitor ];
do

echo \<group ns=\"robot_$i\"\> >> ugbots_ros/launch/world.launch #### WORKER-> VISITOR
echo \<node pkg=\"ugbots_ros\" name=\"visitornode\" type=\"WORKER\"\/\> >> ugbots_ros/launch/world.launch 
echo \<\/group\> >> ugbots_ros/launch/world.launch

echo worker\(pose [ 3.5 $((1+$(($v * 2)))) 0 0 ] name \"V$v\" color \"black\" \) >> world/config/workerinstances.inc
v=$(($v+1))
i=$(($i+1))

done

while [ $d -lt $dog ];
do

echo \<group ns=\"robot_$i\"\> >> ugbots_ros/launch/world.launch
echo \<node pkg=\"ugbots_ros\" name=\"dognode\" type=\"DOG\"\/\> >> ugbots_ros/launch/world.launch 
echo \<\/group\> >> ugbots_ros/launch/world.launch

echo dog\( pose [ 0 $((-1-$(($d * 2)))) 0 0 ] origin [ 0 0 0 270 ] name \"D$d\" color \"brown\" \) >> world/config/animalinstances.inc
d=$(($d+1))
i=$(($i+1))

done

while [ $po -lt $possum ];
do

echo \<group ns=\"robot_$i\"\> >> ugbots_ros/launch/world.launch #### DOG -> POSSUM
echo \<node pkg=\"ugbots_ros\" name=\"possumnode\" type=\"DOG\"\/\> >> ugbots_ros/launch/world.launch 
echo \<\/group\> >> ugbots_ros/launch/world.launch

echo dog\( pose [ 3.5 $((-1-$(($c * 2)))) 0 0 ] name \"P$po\" color \"brown\" \) >> world/config/animalinstances.inc
po=$(($po+1))
i=$(($i+1))

done

: <<'Tractor_Code'
while [ $t -lt $tractor ];
do

echo \<group ns=\"robot_$i\"\> >> ugbots_ros/launch/world.launch
echo \<node pkg=\"ugbots_ros\" name=\"tractornode\" type=\"TRACTOR\"\/\> >> ugbots_ros/launch/world.launch 
echo \<\/group\> >> ugbots_ros/launch/world.launch

echo tractor\( pose [ 0 $((-1-$(($t * 2)))) 0 0 ] name \"T$t\" color \"brown\" \) >> world/config/tractorinstances.inc
t=$(($t+1))
i=$(($i+1))

done
Tractor_Code

echo  \<\/launch\> >> ugbots_ros/launch/world.launch
