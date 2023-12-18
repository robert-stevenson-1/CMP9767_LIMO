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
source_assingment_cmd="source $mount_path/University/Lincoln/MSc/Robot-Programming/CMP9767_LIMO/assignment-ws/install/setup.bash"
$source_assingment_cmd

# create a alias to kill these sessions
alias kill_sim="tmux kill-session -t gazebo_sim"
alias kill_nav="tmux kill-session -t navigation"
# kill all the sessions
alias kill_all_tmux="kill_sim; kill_nav"

# function to restart the simulator (or create it if it doesn't exist)
function restart_sim {
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
    # sim_cmd="ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=$mount_path/University/Lincoln/MSc/Robot-Programming/CMP9767_LIMO/limo_ros/src/limo_gazebosim/worlds/potholes_simple.world"
    sim_cmd="ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=$mount_path/University/Lincoln/MSc/Robot-Programming/CMP9767_LIMO/assignment-ws/src/worlds/potholes_simple.world"
    
    tmux send-keys -t gazebo_sim "$sim_cmd" Enter
}

# call the restart_sim function
restart_sim

#export the restart_sim function
export -f restart_sim

# function to restart the navigation (or create it if it doesn't exist)
function restart_nav {
    # check if we have a navigation tmux session
    tmux has-session -t navigation 2>/dev/null
    if [ $? != 0 ]; then
        # start the simulator in it's own tmux session if it doesn't exist
        echo "Starting navigation session"
        tmux new-session -s navigation -d
        else
        # kill the old sessions and restart a new one
        echo "killed old navigation session"
        kill_nav
        tmux new-session -s navigation -d
    fi
    
    #source limo in the session
    tmux send-keys -t navigation "$source_limo_cmd" Enter
    
    # navigation_launch_cmd="ros2 launch limo_navigation limo_navigation.launch.py use_sim_time:=true map:=$mount_path/University/Lincoln/MSc/Robot-Programming/CMP9767_LIMO/assignment-ws/src/maps/potholes_20mm.yaml params_file:=$mount_path/University/Lincoln/MSc/Robot-Programming/CMP9767_LIMO/assignment-ws/src/params/nav2_params.yaml"
    navigation_launch_cmd="ros2 launch limo_navigation limo_navigation_assignment.launch.py use_sim_time:=true map:=$mount_path/University/Lincoln/MSc/Robot-Programming/CMP9767_LIMO/assignment-ws/src/maps/potholes_20mm.yaml params_file:=$mount_path/University/Lincoln/MSc/Robot-Programming/CMP9767_LIMO/assignment-ws/src/params/nav2_params.yaml"
    
    tmux send-keys -t navigation "$navigation_launch_cmd" Enter
}

# call the restart_nav function
restart_nav

#export the restart_nav function
export -f restart_nav