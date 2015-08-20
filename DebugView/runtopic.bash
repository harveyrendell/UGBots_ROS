#!/bin/bash
source ~/.bashrc

roscd

sleep 5

rostopic echo -c rosout
