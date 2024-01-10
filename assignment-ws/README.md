# Assignment Solution Setup Instructions

## Solution Summary

Using OpenCV on a ROS-enabled robot to detect 'pink potholes' in the environment's road we process image and depth data from the cameras, record the position of each pothole, and published them back to the ROS network in PoseArray.
We applied techniques learned from courses, including additional techniques for filtering duplicate pothole locations using “Euclidean distance” and a defined filter radius.
We also restricted recording pothole locations based on their depth values within a specific range to improve accuracy and reliability.

## Running the solution

### Requirements

Installation guide from the [CMP9767 Repo](https://github.com/LCAS/CMP9767_LIMO/wiki/Simulator-Setup#native).


## Original Git Repo

To clone the original repo (and the submodules required):

```text
git clone --recurse-submodules https://github.com/robert-stevenson-1/CMP9767_LIMO.git
```