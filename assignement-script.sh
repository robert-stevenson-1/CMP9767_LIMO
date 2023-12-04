#!/bin/bash

# source the limo environment
source /media/robert/Portable-Drive/University/Lincoln/MSc/Robot-Programming/CMP9767_LIMO/limo_ros/install/setup.bash

# create a alias to kill these sessions
alias kill_sim="tmux kill-session -t gazebo_sim"
alias kill_rvis="tmux kill-session -t rvis2"
# kill all the sessions
alias kill_all_tmux="kill_sim; kill_rvis"

# try to kill the sessions if they exist
kill_all_tmux

# cmd to launch the simulator world
sim_cmd="ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=/media/robert/Portable-Drive/University/Lincoln/MSc/Robot-Programming/CMP9767_LIMO/limo_ros/src/limo_gazebosim/worlds/potholes_simple.world"

# start the simulator in it's own tmux session
tmux new-session -s gazebo_sim -d
tmux send-keys -t gazebo_sim "$sim_cmd" Enter

# start the rvis2 for the simulator
rvis_file="/media/robert/Portable-Drive/University/Lincoln/MSc/Robot-Programming/CMP9767_LIMO/limo_ros/src/limo_gazebosim/rviz/urdf.rviz"
# rvis command
rvis_cmd="rviz2 -d $rvis_file"
# start the rvis2 in it's own tmux session
tmux new-session -s rvis2 -d 
tmux send-keys -t rvis2 "$rvis_cmd" Enter
