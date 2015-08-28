#!/bin/bash
source ~/.bashrc

roscd

sleep 7

rostopic echo -c rosout
