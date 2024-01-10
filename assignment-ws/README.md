# Assignment Solution - Robert Stevenson

## Solution Summary

Using OpenCV on a ROS-enabled robot to detect 'pink potholes' in the environment's road we process image and depth data from the cameras, record the position of each pothole, and published them back to the ROS network in PoseArray.
We applied techniques learned from courses, including additional techniques for filtering duplicate pothole locations using “Euclidean distance” and a defined filter radius.
We also restricted recording pothole locations based on their depth values within a specific range to improve accuracy and reliability.

## Setup and running my solution

### Requirements

Installation guide from the [CMP9767 Repo](https://github.com/LCAS/CMP9767_LIMO/wiki/Simulator-Setup#native).

### Starting the Simulator, Navigator and Detector

1. Will need to build `limo_ros` by navigating into the the `limo_ros` directory and entering `colcon build`

2. Once built successfully (you may have to do it twice as the `astra camera` package can cause an error on the first build and disappears after building for a second time) source the newly built ROS package

    2.1 Sourcing the package can be done with `source install/setup.bash`

3. You can now go to the `assignment-ws` directory and run `colcon build` to build the assignment related ROS package. Make sure to source the built package like you did before.

4. Start the Simulator with the use of the command: `ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=<path-to-assignment-code>/CMP9767_LIMO_Robert_Stevenson/assignment-ws/src/worlds/potholes_simple.world`

    4.1 Make sure to replace `<path-to-assignment-code>` with the correct path to where you have this codebase

5. In new terminal instance (make sure to source the packages again) we will launch `rviz2` for our simulated environment with the command: `ros2 launch limo_navigation limo_navigation_assignment.launch.py use_sim_time:=true map:=<path-to-assignment-code>/CMP9767_LIMO_Robert_Stevenson/assignment-ws/src/maps/potholes_20mm.yaml params_file:=<path-to-assignment-code>/CMP9767_LIMO_Robert_Stevenson/assignment-ws/src/params/nav2_params.yaml`

    5.1 Make sure to replace `<path-to-assignment-code>` with the correct path to where you have this codebase

    5.2 From here you can load up the `limo_navigation_assignment.rviz` file: `<path-to-assignment-code>/CMP9767_LIMO_Robert_Stevenson/limo_ros/src/limo_navigation/rviz/limo_navigation_assignment.rviz`. This will give you the visualisation of the detected pothole locations on the SLAM map.
    (Additionally, there are some helpful visualisation if you require them for debugging.)

6. In new terminal instance (make sure to source the packages again) we must first start the pothole detector node. This can be done with the command: `ros2 run assignment pothole_detector`

    6.1 You will get to new image window appear. These are preview of the depth camera view and an annotated version of the RGB camera. The `blue contours` that are drawn round the pothole are the detected potholes based in the mask. The `small green dots` are the centroid of the potholes who's positions calculated and recorded.

7. Finally, In new terminal instance (make sure to source the packages again) we can launch the `navigator` node for the robot with the command: `ros2 run assignment navigator`

8. The robot should now start navigating through the world via the navigator's waypoint and be detecting publishing the detected potholes found during the travel. The detect pothole position can be visualise in `rviz2` via a `PoseArray`, and the total count can be found in the detector node's terminal or on the annotated image preview.

## Original Git Repo (if required)

To clone my [original repo](https://github.com/robert-stevenson-1/CMP9767_LIMO.git) (and the git submodules required):

```text
git clone --recurse-submodules https://github.com/robert-stevenson-1/CMP9767_LIMO.git
```
