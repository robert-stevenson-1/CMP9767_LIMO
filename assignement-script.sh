#!/bin/bash

# source the limo environment
source /media/robert/Portable-Drive/University/Lincoln/MSc/Robot-Programming/CMP9767_LIMO/limo_ros/install/setup.bash

# add an alise to launch the simulator world
alias sim_start="ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=/media/robert/Portable-Drive/University/Lincoln/MSc/Robot-Programming/CMP9767_LIMO/limo_ros/src/limo_gazebosim/worlds/potholes_simple.world"