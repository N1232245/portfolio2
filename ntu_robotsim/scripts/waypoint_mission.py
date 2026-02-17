#!/usr/bin/env python3
"""
Waypoint mission navigator using Nav2 Simple Commander API.
"""

import time
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def create_pose(x: float, y: float, theta: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(theta / 2.0)
    pose.pose.orientation.w = math.cos(theta / 2.0)
    return pose


def main() -> None:
    rclpy.init()
    navigator = BasicNavigator()

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Waypoints (x, y, theta)
    waypoints = [
        create_pose(2.0985865592956543, 3.0616133213043213, 0.0),    # WP1
        create_pose(-0.20121914148330688, 3.9527974128723145, 0.0),  # WP2
        create_pose(3.42097544670105, 2.687889337539673, 0.0),       # WP3
        create_pose(-4.08214807510376, 3.4066073894500732, 0.0),     # WP4
        create_pose(3.047246217727661, -0.07188021391630173, 0.0),   # WP5
    ]

    print(f"Starting waypoint mission with {len(waypoints)} waypoints...")

    mission_start = time.time()

    for i, waypoint in enumerate(waypoints, 1):
        print(f"\nNavigating to waypoint {i}/{len(waypoints)}")
        print(f"  Position: ({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f})")

        waypoint_start = time.time()
        navigator.goToPose(waypoint)

        while not navigator.isTaskComplete():
            time.sleep(0.1)

        waypoint_time = time.time() - waypoint_start
        result = navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            print(f"  ✓ Reached waypoint {i} in {waypoint_time:.1f} seconds")
        elif result == TaskResult.CANCELED:
            print(f"  ✗ Waypoint {i} was canceled (aborting)")
            break
        elif result == TaskResult.FAILED:
            print(f"  ✗ Failed to reach waypoint {i} (aborting)")
            break

        time.sleep(1.0)

    mission_time = time.time() - mission_start
    print(f"\nMission completed in {mission_time:.1f} seconds")

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
