#!/bin/bash
 
echo -e "Number of kiwifruit rows to be generated: \c "
read  rows

echo -e "Number of picker robots to be generated: \c "
read picker

echo -e "Number of carrier robots to be generated: \c "
read carrier

rm worldinstances.inc

echo include \"kiwirow.inc\" > worldinstances.inc

i=0
j=0

while [ $i -lt $rows ];
do
k=$((-20+$j))
echo rows\(\pose [ $k 20 0 0 ]\)\ >> worldinstances.inc
i=$(($i+1))
j=$((8+$j))

done

rosrun stage_ros stageros ./world/myworld.world
