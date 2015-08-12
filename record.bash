#!/bin/bash
 
#Create rosbag folder in current folder
mkdir ./bagfiles
cd bagfiles

read -p "would you like to record session?" choice

case "$choice" in
	y|Y)
	echo "Enter in seconds how long you would like to record for: "
	read seconds

	echo "Enter name of output file: "
	read name

	#Begin recording and kill after n seconds

	( cmdpid=$BASHPID; (sleep $seconds; kill -INT $cmdpid) & exec rosbag record -a -O $name)
	pkill rosbag
	pkill record
		

	rosbag reindex $name.*


	rm *.orig.*;;
	
	

esac
