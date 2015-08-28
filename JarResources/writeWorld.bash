#!/bin/bash
source ~/.bashrc
roscd

#assign parameters to appropriate variables
picker=$1
carrier=$2
worker=$3
visitor=$4
dog=$5
cat=$6
possum=$7
tractor=$8
kiwitree=$9

#make a directory to store all the instances file to
mkdir -p world/config

#remove all previous files if they exist
rm ugbots_ros/launch/world.launch
rm world/config/robotinstances.inc
rm world/config/peopleinstances.inc
rm world/config/animalinstances.inc
rm world/config/tractorinstances.inc
rm world/config/treeinstances.inc
rm world/config/beaconinstances.inc

#initialise each instance file by adding the model include statement
echo  \<launch\> > ugbots_ros/launch/world.launch
echo include \"models\/cmdCenter.inc\" >> world/config/beaconinstances.inc
echo include \"models\/beaconcore.inc\" >> world/config/beaconinstances.inc
echo include \"models\/kiwirow.inc\" >> world/config/treeinstances.inc
echo include \"models\/robots.inc\" >> world/config/robotinstances.inc
echo include \"models\/workers.inc\" >> world/config/peopleinstances.inc
echo include \"models\/dogs.inc\" >> world/config/animalinstances.inc
echo include \"models\/cats.inc\" >> world/config/animalinstances.inc
echo include \"models\/possums.inc\" >> world/config/animalinstances.inc
echo include \"models\/visitors.inc\" >> world/config/peopleinstances.inc
echo include \"models\/tractors.inc\" >> world/config/tractorinstances.inc

number=0
beacon=$(($kiwitree+$kiwitree-1))
i=0
j=0
w=0
a=0
v=0
d=0
c=0
po=0
t=0
rand=0
rand2=0
tree=0
treenum=0
z=0

#Creating Core Unit ****
#echo launch code into launch file
echo \<group ns=\"robot_$number\"\> >> ugbots_ros\/launch\/world.launch
echo \<node pkg=\"ugbots_ros\" name=\"core\" type=\"CORE\"\/\> >> ugbots_ros\/launch\/world.launch 
echo \<\/group\> >> ugbots_ros\/launch\/world.launch

#write position of command center into instances file
echo cmdCenter\( pose [ 40 40  0 0 ] origin [ 0 0 0 270 ] name \"core\" color \"DimGrey\" \) >> world/config/beaconinstances.inc

number=$(($number+1))


#generating kiwifruit rows ***
#generating odd number of instances of kiwifruit rows
if (( $kiwitree % 2 )); then
    #create one row in center
    echo rows \(pose [ 0 -35 -1.002 0 ]\) >> world/config/treeinstances.inc
    treenum=$(($treenum+1))
    tree=$(($tree+1))
    while [ $tree -lt $kiwitree ];
    do
        #create two rows on either side until specified number of trees reached
        left=$(echo "scale=2; 0-$treenum*3.5" | bc)
        right=$(echo "scale=2; 0+$treenum*3.5" | bc)
        echo rows \(pose [ $left -35 -1.002 0 ]\) >> world/config/treeinstances.inc
        echo rows \(pose [ $right -35 -1.002 0 ]\) >> world/config/treeinstances.inc

        bleft=$(echo "scale=2; -1.75-$z*3.5" | bc)
        bright=$(echo "scale=2; 1.75+$z*3.5" | bc)
        
        #write launch code for beacons into launch file and create beacons on both ends of the rows
        echo \<group ns=\"robot_$number\"\> >> ugbots_ros\/launch\/world.launch
		echo \<node pkg=\"ugbots_ros\" name=\"beacon\" type=\"BEACON\"\/\> >> ugbots_ros\/launch\/world.launch 
		echo \<\/group\> >> ugbots_ros\/launch\/world.launch
        echo point\( pose [ $bleft 38 0 0 ] name \"robot_$number\" color \"yellow\" \) >> world/config/beaconinstances.inc
        number=$(($number+1))
        
        echo \<group ns=\"robot_$number\"\> >> ugbots_ros\/launch\/world.launch
		echo \<node pkg=\"ugbots_ros\" name=\"beacon\" type=\"BEACON\"\/\> >> ugbots_ros\/launch\/world.launch 
		echo \<\/group\> >> ugbots_ros\/launch\/world.launch
        echo point\( pose [ $bright 38 0 0 ] name \"robot_$number\" color \"yellow\" \) >> world/config/beaconinstances.inc
        number=$(($number+1))
        
        echo \<group ns=\"robot_$number\"\> >> ugbots_ros\/launch\/world.launch
		echo \<node pkg=\"ugbots_ros\" name=\"beacon\" type=\"BEACON\"\/\> >> ugbots_ros\/launch\/world.launch 
		echo \<\/group\> >> ugbots_ros\/launch\/world.launch
        echo point\( pose [ $bleft -38 0 0 ] name \"robot_$number\" color \"yellow\" \) >> world/config/beaconinstances.inc
        number=$(($number+1))
        
        echo \<group ns=\"robot_$number\"\> >> ugbots_ros\/launch\/world.launch
		echo \<node pkg=\"ugbots_ros\" name=\"beacon\" type=\"BEACON\"\/\> >> ugbots_ros\/launch\/world.launch 
		echo \<\/group\> >> ugbots_ros\/launch\/world.launch
        echo point\( pose [ $bright -38 0 0 ] name \"robot_$number\" color \"yellow\" \) >> world/config/beaconinstances.inc
        number=$(($number+1))
        
        tree=$(($tree+2))
        treenum=$(($treenum+1))
        z=$(($z+1))
    done
#generating even number of rows of kiwifruit rows
elif ! (( $kiwitree % 2 )); then
    while [ $tree -lt $kiwitree ];
    do
        left=$(echo "scale=2; -1.75-$treenum*3.5" | bc)
        right=$(echo "scale=2; 1.75+$treenum*3.5" | bc)

        #generate two rows at a time until specified number is reached
        echo rows \(pose [ $left -35 -1.002 0 ]\) >> world/config/treeinstances.inc
        echo rows \(pose [ $right -35 -1.002 0 ]\) >> world/config/treeinstances.inc

        bleft=$(echo "scale=2; 0-$z*3.5" | bc)
        bright=$(echo "scale=2; 0+$z*3.5" | bc)
	
    #write launch code for beacons into launch file and create beacons on both ends of the kiwifruit rows
	if !(($tree==0)); then
        echo \<group ns=\"robot_$number\"\> >> ugbots_ros\/launch\/world.launch
		echo \<node pkg=\"ugbots_ros\" name=\"beacon\" type=\"BEACON\"\/\> >> ugbots_ros\/launch\/world.launch 
		echo \<\/group\> >> ugbots_ros\/launch\/world.launch
        echo point\( pose [ $bleft 38 0 0 ] name \"robot_$number\" color \"yellow\" \) >> world/config/beaconinstances.inc
        number=$(($number+1))
	fi

        echo \<group ns=\"robot_$number\"\> >> ugbots_ros\/launch\/world.launch
		echo \<node pkg=\"ugbots_ros\" name=\"beacon\" type=\"BEACON\"\/\> >> ugbots_ros\/launch\/world.launch 
		echo \<\/group\> >> ugbots_ros\/launch\/world.launch
        echo point\( pose [ $bright 38 0 0 ] name \"robot_$number\" color \"yellow\" \) >> world/config/beaconinstances.inc
        number=$(($number+1))

	if !(($tree==0)); then
        echo \<group ns=\"robot_$number\"\> >> ugbots_ros\/launch\/world.launch
		echo \<node pkg=\"ugbots_ros\" name=\"beacon\" type=\"BEACON\"\/\> >> ugbots_ros\/launch\/world.launch 
		echo \<\/group\> >> ugbots_ros\/launch\/world.launch
        echo point\( pose [ $bleft -38 0 0 ] name \"robot_$number\" color \"yellow\" \) >> world/config/beaconinstances.inc
        number=$(($number+1))
	fi

        echo \<group ns=\"robot_$number\"\> >> ugbots_ros\/launch\/world.launch
		echo \<node pkg=\"ugbots_ros\" name=\"beacon\" type=\"BEACON\"\/\> >> ugbots_ros\/launch\/world.launch 
		echo \<\/group\> >> ugbots_ros\/launch\/world.launch
        echo point\( pose [ $bright -38 0 0 ] name \"robot_$number\" color \"yellow\" \) >> world/config/beaconinstances.inc
	number=$(($number+1))
     
	tree=$(($tree+2))
        treenum=$(($treenum+1))
        z=$(($z+1))
    done
fi

#creating picker robots
temp=0

while [ $i -lt $picker ];
do

#write launch code for picker robots into launch file
echo \<group ns=\"robot_$number\"\> >> ugbots_ros\/launch\/world.launch
echo \<node pkg=\"ugbots_ros\" name=\"robotnode\" type=\"PICKER\"\/\> >> ugbots_ros\/launch\/world.launch 
echo \<\/group\> >> ugbots_ros\/launch\/world.launch


if !(($i%2)); then
	temp=$(($temp+1))
fi

#write instance of picker robot into instance file
#position picker robots in a line at the bottom
posx=$((-40 + (( $(($i%2)) * 8 )) ))
posy=$((-50 + $(($temp * 4))))
echo pickerRobot\(pose [ $posx $posy 0 0 ]\ name \"P$i\" color \"red\"\) >> world/config/robotinstances.inc

i=$(($i+1))
number=$(($number+1))

done

#creating carrier robots
while [ $j -lt $carrier ];
do
# write launch code for carrier robots into launch file
echo \<group ns=\"robot_$number\"\> >> ugbots_ros\/launch\/world.launch
echo \<node pkg=\"ugbots_ros\" name=\"robotnode\" type=\"CARRIER\"\/\> >> ugbots_ros\/launch\/world.launch
echo \<\/group\> >> ugbots_ros\/launch\/world.launch

#write instance of carrier robot into instance file
echo carrierRobot\(pose [ $((25 + (($j*5)))) -15 0 270 ] name \"C$j\" color \"blue\"\) >> world/config/robotinstances.inc

j=$(($j+1))
number=$(($number+1))

done

#creating worker nodes
while [ $w -lt $worker ];
do

rand=55
rand2=-40

#write launch code for workers into launch file
echo \<group ns=\"robot_$number\"\> >> ugbots_ros/launch/world.launch
echo \<node pkg=\"ugbots_ros\" name=\"workernode\" type=\"WORKER\"\/\> >> ugbots_ros/launch/world.launch 
echo \<\/group\> >> ugbots_ros/launch/world.launch

#write instance of workers into instance file
#workers are spawned near the entrance of the kiwifruit orchard
echo worker\(pose [ $rand $rand2 0 270 ] origin [ 0 0 0 90 ] name \"W$w\"\) >> world/config/peopleinstances.inc
w=$(($w+1))
number=$(($number+1))

done

visitorx=58
visitory=-47.5

#creating visitor nodes
while [ $v -lt $visitor ];
do

rand=$visitorx
rand2=$visitory

#write launch code for visitors into launch file
echo \<group ns=\"robot_$number\"\> >> ugbots_ros/launch/world.launch 
echo \<node pkg=\"ugbots_ros\" name=\"visitornode\" type=\"VISITOR\"\/\> >> ugbots_ros/launch/world.launch 
echo \<\/group\> >> ugbots_ros/launch/world.launch

#write instance of visitors into instance file
#visitors are spawned at the entrance of the kiwifruit orchard
echo visitor\(pose [ $rand $rand2 0 180 ] origin [ 0 0 0 90 ] name \"V$v\" \) >> world/config/peopleinstances.inc
v=$(($v+1))
number=$(($number+1))
visitorx=$(($rand+5))

done

#creating dogs
while [ $d -lt $dog ];
do

rand=$(( (RANDOM % 97) - 46 )) 

#dogs are spawned at random places in the kiwifruit orchard
if (($rand>=-12 && $rand<=12));
then
    rand3=$(( (RANDOM % 15) - 39 )) 
    rand4=$(( (RANDOM % 15) + 25 ))
    if [ $(( (RANDOM % 2) + 1 )) -lt "2" ];
    then
        rand2=$rand3
    else
        rand2=$rand4
    fi
else
    rand2=$(( (RANDOM % 99) - 48 )) 
fi 
#write launch code for dogs into launch file
echo \<group ns=\"robot_$number\"\> >> ugbots_ros/launch/world.launch
echo \<node pkg=\"ugbots_ros\" name=\"dognode\" type=\"DOG\"\/\> >> ugbots_ros/launch/world.launch 
echo \<\/group\> >> ugbots_ros/launch/world.launch

#write instance of dogs into instance file
echo dog\( pose [ $rand $rand2 0 0 ] origin [ 0 0 0 270 ] name \"D$d\" \) >> world/config/animalinstances.inc
d=$(($d+1))
number=$(($number+1))

done

#creating cat
while [ $c -lt $cat ];
do

rand=$(( (RANDOM % 80) - 40 )) 

#write launch code for cats into launch file
echo \<group ns=\"robot_$number\"\> >> ugbots_ros/launch/world.launch
echo \<node pkg=\"ugbots_ros\" name=\"catnode\" type=\"CAT\"\/\> >> ugbots_ros/launch/world.launch 
echo \<\/group\> >> ugbots_ros/launch/world.launch

#write instance of cat into instance file
echo cats\( pose [ $rand 47 0 0 ] origin [ 0 0 0 270 ] name \"C$c\" \) >> world/config/animalinstances.inc
c=$(($c+1))
number=$(($number+1))

done

#creating possums
while [ $po -lt $possum ];
do

#write launch code for possums in the launch file
echo \<group ns=\"robot_$number\"\> >> ugbots_ros/launch/world.launch 
echo \<node pkg=\"ugbots_ros\" name=\"possumnode\" type=\"POSSUM\"\/\> >> ugbots_ros/launch/world.launch 
echo \<\/group\> >> ugbots_ros/launch/world.launch

#possums are spawned near the kiwifruit trees
possumStart=$(echo "scale=2; $bleft-1.75" | bc)

index=$(( (RANDOM % 28) -14))

yPos=$(echo "scale=2; $index*2.5 -1.25"| bc)

#write instance of possum into instance file
echo possum\( pose [ $left $yPos 0 0 ] origin [ 0 0 0 270 ] name \"PO$po\" color \"wheat4\" \) >> world/config/animalinstances.inc

po=$(($po+1))
number=$(($number+1))

done

#write instance of tractor into instance file
echo tractor\( pose [ -45 0 0 0 ] origin [ 0 0 0 270] name \"T$t\" \) >> world/config/tractorinstances.inc


t=$(($t+1))
number=$(($number+1))

done

echo  \<\/launch\> >> ugbots_ros/launch/world.launch
