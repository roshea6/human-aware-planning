#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np

class MapSolver(Node):
    def __init__(self):
        super().__init__('navigation_node')

        self.path_pub = self.create_publisher(Path, 'custom_path', 10)

        # TODO: Add subscribers to the current turtlebot pose and the nav goal so we can do our path planning that way


    def pubPath(self):
        start_x = -2.0
        end_x = 1.0
        start_y = -0.5
        end_y = -0.5

        int_x = np.linspace(start_x, end_x, 25)
        int_y = np.linspace(start_y, end_y, 25)

        # Create blank path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        waypoints = []

        for (x, y) in zip(int_x, int_y):
            pose_stamped = PoseStamped()
            # pose_stamped.header.stamp = self.get_clock().now().to_msg()
            # pose_stamped.header.frame_id = 'map'  # Set the frame ID
            pose_stamped.pose.position.x = x  # Set x-coordinate
            pose_stamped.pose.position.y = y  # Set y-coordinate
            pose_stamped.pose.position.z = 0.0   # Set z-coordinate
            waypoints.append(pose_stamped)
        # Populate the Path message with the waypoints
        path_msg.poses = waypoints

        print("Publishing")
        # print(path_msg)

        # Publish the Path message
        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MapSolver()
    try:
        node.pubPath()
    finally:
        node.destroy_node()
        rclpy.shutdown()    

if __name__ == "__main__":
    main()
