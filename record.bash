#!/bin/bash
 
#Create rosbag folder in current folder
mkdir ./bagfiles
cd bagfiles

echo "Enter in seconds how long you would like to record for"
read seconds

#Begin recording and kill after n seconds

( cmdpid=$BASHPID; (sleep "$seconds"; kill -TERM $cmdpid) & exec rosbag record -a )

cd ..

