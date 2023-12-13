#!/bin/bash

# check the mount point to see if we are via linux native or via WSL and set the mount path variable
if [ -e "/mnt/e/University/Lincoln/MSc/Robot-Programming/CMP9767_LIMO/assignement-script.sh" ]; then
    mount_path="/mnt/e/"
else
    mount_path="/media/robert/Portable-Drive/"
fi


# source the limo environment
source_limo_cmd="source $mount_path/University/Lincoln/MSc/Robot-Programming/CMP9767_LIMO/limo_ros/install/setup.bash"
$source_limo_cmd

# create a alias to kill these sessions
alias kill_sim="tmux kill-session -t gazebo_sim"
alias kill_rviz="tmux kill-session -t rviz2"
alias kill_nav="tmux kill-session -t navigation"
# kill all the sessions
alias kill_all_tmux="kill_sim; kill_rviz; kill_nav"

# check if the gazebo tmux session exists
tmux has-session -t gazebo_sim 2>/dev/null
if [ $? != 0 ]; then
    # start the simulator in it's own tmux session if it doesn't exist
    echo "Starting gazebo_sim session"
    tmux new-session -s gazebo_sim -d
    else
    echo "killed old gazebo_sim session"
    # kill the old sessions and restart a new one
    kill_sim
    tmux new-session -s gazebo_sim -d
fi
# cmd to launch the simulator world
sim_cmd="ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=$mount_path/University/Lincoln/MSc/Robot-Programming/CMP9767_LIMO/limo_ros/src/limo_gazebosim/worlds/potholes_simple.world"
tmux send-keys -t gazebo_sim "$sim_cmd" Enter

# check if the rviz tmux session exists
# tmux has-session -t rviz2 2>/dev/null
# if [ $? != 0 ]; then
#     # start the simulator in it's own tmux session if it doesn't exist
#     echo "Starting rviz session"
#     tmux new-session -s rviz2 -d
#     else
#     # kill the old sessions and restart a new one
#     kill_rviz
#     echo "killed old rviz2 session"
#     tmux new-session -s rviz2 -d
#     # start the rviz2 for the simulator
# fi
# rviz_file="$mount_path/University/Lincoln/MSc/Robot-Programming/CMP9767_LIMO/limo_ros/src/limo_gazebosim/rviz/urdf.rviz"
# # rviz command
# rviz_cmd="rviz2 -d $rviz_file"
# tmux send-keys -t rviz2 "$rviz_cmd" Enter

# check if we have a navigation tmux session
tmux has-session -t navigation 2>/dev/null
if [ $? != 0 ]; then
    # start the simulator in it's own tmux session if it doesn't exist
    echo "Starting navigation session"
    tmux new-session -s navigation -d
    else
    # kill the old sessions and restart a new one
    echo "killed old navigation session"
    kill_all_tmux
    tmux new-session -s navigation -d
fi
#source limo in the session
tmux send-keys -t navigation "$source_limo_cmd" Enter
navigation_launch_cmd="ros2 launch limo_navigation limo_navigation.launch.py map:=/media/robert/Portable-Drive/University/Lincoln/MSc/Robot-Programming/CMP9767_LIMO/limo_ros/src/limo_navigation/maps/assessment_map.yaml"
tmux send-keys -t navigation "$navigation_launch_cmd" Enter