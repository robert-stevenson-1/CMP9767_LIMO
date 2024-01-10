##############
# navigator.py
#
# Author: Robert Stevenson
# Date: 28-12-2023
#
# This file contains the code for the a simple navigator that follows a set of waypoint
# goals.
# This is based on the example found in the nav2 documentation with a few modifications to
# suit the purpose of this project/assignment goals, such as reading the goals list from
# a file.
#
# Example source: https://github.com/ros-planning/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/example_waypoint_follower.py
##############

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from copy import deepcopy


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # set the init pose
    # origin: [-1.51, -1.32, 0]
    init_pose = PoseStamped()
    init_pose.header.frame_id = "map"
    init_pose.header.stamp = navigator.get_clock().now().to_msg()
    init_pose.pose.position.x = 0.0
    init_pose.pose.position.y = 0.0
    init_pose.pose.orientation.z = -0.000398603
    init_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(init_pose)

    # TODO: read the goals from a file
    waypoints_coords = [
        [1.05, -0.17, 0.0], # Orientation(0, 0, -0.709153, 0.705054)
        [1.05, -0.92, 0.0], # Orientation(0, 0, -0.99999, 0.00440896)
        [-1.05, -0.92, 0.0], # Orientation(0, 0, 0.706825, 0.707388)
        [-1.05, -0.17, 0.0], # Orientation(0, 0, -0.000398603, 1)
        [0.0, 0.0, 0.0],
    ]

    waypoint_orientations = [
        [0.0, 0.0, -0.709153, 0.705054],
        [0.0, 0.0, -0.99999, 0.00440896],
        [0.0, 0.0, 0.706825, 0.707388],
        [0.0, 0.0, -0.000398603, 1.0],
        [0.0, 0.0, -0.000398603, 1.0],
    ]

    waypoint_data = (waypoints_coords, waypoint_orientations)

    # wait for the navigation to fully activate
    navigator.waitUntilNav2Active()

    #  while the node is running
    while rclpy.ok():

        # send our path of points
        goal_points = []
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.orientation.z = 0.707
        goal_pose.pose.orientation.w = 0.707

        # for each waypoint
        for coord, orientation in zip(waypoint_data[0], waypoint_data[1]):
            goal_pose.pose.orientation.x = orientation[0]
            goal_pose.pose.orientation.y = orientation[1]
            goal_pose.pose.orientation.z = orientation[2]
            goal_pose.pose.orientation.w = orientation[3]
            goal_pose.pose.position.x = coord[0]
            goal_pose.pose.position.y = coord[1]
            goal_points.append(deepcopy(goal_pose))

        # start the navigation
        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(goal_points)

        # TODO: do stuff while navigating if required. (eg. print current waypoint)
        point_index = 0
        while not navigator.isTaskComplete():
            point_index = point_index + 1
            feedback = navigator.getFeedback()
            if feedback and point_index % 5 == 0:
                print(
                    "Current Waypoint: "
                    + str(feedback.current_waypoint + 1)
                    + "/"
                    + str(len(goal_points))
                )

        # TODO: do stuff when navigation is complete
        nav_result = navigator.getResult()
        if nav_result == TaskResult.SUCCEEDED:
            print("Navigation Succeeded")
        elif nav_result == TaskResult.FAILED:
            print("Navigation Failed")
        elif nav_result == TaskResult.CANCELED:
            print("Navigation Canceled")
        else:
            print("Navigation Result Unknown")

        # shutdown the navigator
        navigator.lifecycleShutdown()
        exit(0)


if __name__ == "__main__":
    main()
