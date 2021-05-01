#!/bin/bash

#Build the project
cd ~/..
catkin_make
echo "source devel/setup.bash"  >> ~/.bashrc
