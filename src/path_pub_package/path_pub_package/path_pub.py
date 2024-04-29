#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
import cv2
import time
import math
import copy
from queue import PriorityQueue

# class MapSolver(Node):
#     def __init__(self):
#         super().__init__('navigation_node')

#         self.path_pub = self.create_publisher(Path, 'custom_path', 10)

#         # TODO: Add subscribers to the current turtlebot pose and the nav goal so we can do our path planning that way


#     def pubPath(self):
#         start_x = -2.0
#         end_x = 1.0
#         start_y = -0.5
#         end_y = -0.5

#         int_x = np.linspace(start_x, end_x, 25)
#         int_y = np.linspace(start_y, end_y, 25)

#         # Create blank path message
#         path_msg = Path()
#         path_msg.header.stamp = self.get_clock().now().to_msg()
#         path_msg.header.frame_id = 'map'

#         waypoints = []

#         for (x, y) in zip(int_x, int_y):
#             pose_stamped = PoseStamped()
#             # pose_stamped.header.stamp = self.get_clock().now().to_msg()
#             # pose_stamped.header.frame_id = 'map'  # Set the frame ID
#             pose_stamped.pose.position.x = x  # Set x-coordinate
#             pose_stamped.pose.position.y = y  # Set y-coordinate
#             pose_stamped.pose.position.z = 0.0   # Set z-coordinate
#             waypoints.append(pose_stamped)
#         # Populate the Path message with the waypoints
#         path_msg.poses = waypoints

#         print("Publishing")
#         # print(path_msg)

#         # Publish the Path message
#         self.path_pub.publish(path_msg)

class AStarMapSolver(Node):
    def __init__(self, record_video=False, c2g_weight=1, use_lines=False, save_every_n_frames=500):
        super().__init__('a_star_navigation_node')

        # self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # self.path_pub = self.create_publisher(Path, 'custom_path', 10)

        # Define the map colors
        self.map_colors = {"obstacle": [0, 0, 255],
                           "clearance": [0, 255, 0],
                           "unexplored": [0, 0, 0],
                           "explored": [255, 0, 0],
                           "path": [255, 255, 255],
                           "start": [0, 255, 255],
                           "goal": [255, 0, 255]}
        
        self.map_dim = (400, 800)

        self.save_scale = 1
        
        # Used to bin locations to the nearest location to make the search space smaller
        self.search_loc_thresh = 10
        self.search_ang_thresh = 10
        
        # Number of intermediate points to generate for the curve
        self.num_int_points = 10
        self.int_x_exp_vals = (np.linspace(0.0, 1.0, self.num_int_points)**2)
        self.int_y_exp_vals = (np.linspace(0.0, 1.0, self.num_int_points))
        
        self.int_check_points = np.array([0.25, 0.5, 0.75])
        
        self.save_every_n_frames = save_every_n_frames
        
        self.c2g_weight = c2g_weight
        # Determine if lines should be drawn from spot to spot instead of just filling in the pixel
        self.use_lines = use_lines
        
        # Clearance in milimeters
        while True:
            self.clearance = int(input("Enter the clearance value in mm (0-10 recommended): "))
            
            if self.clearance < 0 or self.clearance > 10:
                continue
            else:
                break
            
        # Get the two rpms from the user
        while True:
            self.rpm1 = int(input("Enter the first rpm (30-70 recommended): "))
            
            if self.rpm1 < 0 or self.rpm1> 100:
                continue
            else:
                break
        while True:
            self.rpm2 = int(input("Enter the second rpm (30-70 recommended): "))
            
            if self.rpm2 < 0 or self.rpm2> 100:
                continue
            else:
                break
            
        # Robot geometric params
        self.robot_wheel_rad = 3.3 #0.033
        self.robot_rad = 22.0 #.220
        self.robot_wheel_dist =  28.7 #0.287
        
        self.clearance += self.robot_rad
        self.clearance = int(self.clearance)
        
        self.timestep = 0.01
        
        # Distance to goal pose that is acceptable to hit before considering exploration complete
        self.dist_tolerance = 10
        self.angle_tolerance = 30
        
        self.angle_increment = 30
        self.valid_orientations = np.array([idx * self.angle_increment for idx in range(0, int(360/self.angle_increment))])
        
        
        self.action_set = [[0, self.rpm1], 
                           [self.rpm1, 0], 
                           [self.rpm1, self.rpm1], 
                           [0, self.rpm2], 
                           [self.rpm2, 0], 
                           [self.rpm2, self.rpm2], 
                           [self.rpm1, self.rpm2], 
                           [self.rpm2, self.rpm1]]
        
        
        self.world_map = self.makeMap()
        
        self.getStartAndGoalInput()
        
        self.node_index = 0
        
        node = (0, self.node_index, self.start_node, self.start_node, 0, [0, 0])
        
        self.open_list = PriorityQueue()
        
        # Nodes that we've checked in general
        # Will be used to easily reference nodes by their pixel location to keep track of lowest cost for each node
        self.checked_nodes = {str(self.start_node): node}
        self.checked_pixels = set()
        
        # Keep track of the closes nodes with their ids as their dictionary keys
        self.closed_list = {}
        
        # Quickly checkable pixel pairs to avoid double searching
        self.closed_pixels = set()
        
        self.open_list.put(node)
        self.node_index += 1
        
        # Make a map that will be drawn on from the world map
        self.draw_map = copy.copy(self.world_map)
        
        # Draw a square for the start and end nodes
        self.draw_map = cv2.rectangle(self.draw_map, 
                                      (self.start_node[1], self.start_node[0]), 
                                      (self.start_node[1] + 20, self.start_node[0] + 20),
                                      color=self.map_colors["start"],
                                      thickness=-1)
        
        self.draw_map = cv2.rectangle(self.draw_map, 
                                      (self.goal_node[1], self.goal_node[0]), 
                                      (self.goal_node[1] + 20, self.goal_node[0] + 20),
                                      color=self.map_colors["goal"],
                                      thickness=-1)
        
        # Add the starting node to the draw map
        self.draw_map[self.start_node[0], self.start_node[1]] = self.map_colors["explored"]
        
        self.record = record_video
        
        if self.record:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_rec = cv2.VideoWriter('a_star_ryan_oshea_output.mp4', fourcc, 120.0, (int(self.map_dim[1]/self.save_scale), int(self.map_dim[0]/self.save_scale)))
            
            
            write_map = cv2.resize(self.draw_map, (int(self.map_dim[1]/self.save_scale), int(self.map_dim[0]/self.save_scale)))
            self.video_rec.write(write_map)
            
        self.path_pixels = []
        self.path_commands = []
        
        self.goal_found = False
    
    # Makes the map image based on the parameters defined in the assigment
    def makeMap(self):
        # Make an all black map to start
        blank_map = np.zeros((self.map_dim[0], self.map_dim[1], 3), np.uint8)

        # cv2.imshow("Blank", blank_map)
        # cv2.waitKey(0)
        
        
        # Border edges
        # Left wall
        obstacle_map = cv2.rectangle(blank_map, 
                                     (0, 0), 
                                     (self.clearance, self.map_dim[1] - self.clearance), 
                                     color=self.map_colors["clearance"], 
                                     thickness=-1)
        
        # Top wall
        obstacle_map = cv2.rectangle(obstacle_map, 
                                     (0, 0), 
                                     (self.map_dim[1], self.clearance),
                                      thickness=-1, 
                                      color=self.map_colors["clearance"])
        
        # Right wall
        obstacle_map = cv2.rectangle(obstacle_map, 
                                     (self.map_dim[1] - self.clearance, 0), 
                                     (self.map_dim[1], self.map_dim[0]),
                                      thickness=-1, 
                                      color=self.map_colors["clearance"])
        
        # Bottom wall
        obstacle_map = cv2.rectangle(obstacle_map, 
                                     (0, self.map_dim[0] - self.clearance), 
                                     (self.map_dim[1], self.map_dim[0]),
                                      thickness=-1, 
                                      color=self.map_colors["clearance"])
        
        # Draw the clearance first because the obstacles will be contained within them
        # Rectangles
        # Rectangle 1
        # Define the top left and bottom right of the normal rectangle 
        top_left = (400, 100)
        bottom_right = (500, 150)
        
        # First draw the clearance rectangle
        obstacle_map = cv2.rectangle(obstacle_map, 
                                     (top_left[0]-self.clearance, top_left[1] - self.clearance), 
                                     (bottom_right[0] + self.clearance, bottom_right[1] + self.clearance),
                                      thickness=-1, 
                                      color=self.map_colors["clearance"])
        
        # Draw the actual obstacle
        obstacle_map = cv2.rectangle(obstacle_map, 
                                     (top_left[0], top_left[1]), 
                                     (bottom_right[0], bottom_right[1]),
                                      thickness=-1, 
                                      color=self.map_colors["obstacle"])
        
        # cv2.imshow("Map", obstacle_map)
        # cv2.waitKey(0)
        
        return obstacle_map
    
    # Gets the start and end point from user input and stores them
    def getStartAndGoalInput(self):
        # Loop until a valid starting point is input
        while True:
            start_x = int(input("Enter start pixel x value: "))
            start_y = self.map_dim[0] - int(input("Enter start pixel y value: "))
            start_theta = int(input("Enter start theta value that is a multiple of 30: "))
            
            if start_x > self.map_dim[1] or start_x < 0 or start_y > self.map_dim[0] or start_y < 0:
                print("Please choose values inside the bounds of the image")
            elif list(self.world_map[start_y, start_x]) == self.map_colors["obstacle"] or list(self.world_map[start_y, start_x]) == self.map_colors["clearance"]:
                print("Start location entered collides with obstacle. Please enter a new value")
            elif not start_theta % 30 == 0:
                print("Start angle not a multiple of 30")
            else:
                break
            
        # Need to swap y and x because of the way numpy indexes things
        self.start_node = (start_y, start_x, start_theta)
                
        # Loop until a valid goal input is input
        while True:
            goal_x = int(input("Enter goal pixel x value: "))
            goal_y = self.map_dim[0] - int(input("Enter goal pixel y value: "))
            goal_theta = int(input("Enter start theta value that is a multiple of 30: "))
            
            if goal_x > self.map_dim[1] or goal_x < 0 or goal_y > self.map_dim[0] or goal_y < 0:
                print("Please choose values inside the bounds of the image")
            elif list(self.world_map[goal_y, goal_x]) == self.map_colors["obstacle"] or list(self.world_map[goal_y, goal_x]) == self.map_colors["clearance"]:
                print("Goal location entered collides with obstacle. Please enter a new value")
            elif not goal_theta % 30 == 0:
                print("Goal angle not a multiple of 30")
            else:
                break

        while True:
            self.step_size = int(input("Please enter a step size value between 1 and 3: "))

            if self.step_size < 1 or self.step_size > 3:
                print("Invalid step size")
            else:
                # Multiply step by 2 to account for the larger map
                self.step_size *= 2
                break
            
        self.goal_node = (goal_y, goal_x, goal_theta)
            
    # Converts degrees to radians
    def deg2rad(self, deg):
        return (math.pi/180) * deg
    
    def rad2deg(self, rad):
        return rad * 180/math.pi
    
    def applyMoves(self, start_pixel, cost):
        for move in self.action_set:
            valid_line = True
            # print(move)
            current_angle = self.deg2rad(start_pixel[2])
            
            x_vel = self.robot_wheel_rad/2 * (move[0] + move[1]) * math.sin(current_angle)
            y_vel = self.robot_wheel_rad/2 * (move[0] + move[1]) * math.cos(current_angle)
            
            # Calculate new x and y locations based on the calculated x and y velocities
            # Bound these new values to the grid set by the search resolution threshold
            new_x = start_pixel[0] + self.step_size*x_vel*self.timestep
            new_x = int(self.search_loc_thresh * round(new_x/self.search_loc_thresh))
            new_y = start_pixel[1] + self.step_size*y_vel*self.timestep
            new_y = int(self.search_loc_thresh * round(new_y/self.search_loc_thresh))
            
            ang_vel = self.robot_wheel_rad/self.robot_wheel_dist * (move[1] - move[0])

            # print(x_vel, y_vel, ang_vel)

            new_angle = start_pixel[2] + ang_vel * self.timestep * self.step_size * 100
            
            new_angle = new_angle % 360
            
            new_angle = self.search_ang_thresh * round(new_angle/self.search_ang_thresh)
            
            # Calculate the list of intermediate x and y pairs for the curved path
            # Currently just use an exponential curve between the start and end point which looks correct
            x_diff = new_x - start_pixel[0]
            y_diff = new_y - start_pixel[1]
            
            int_x_list = start_pixel[0] + (x_diff * self.int_x_exp_vals)
            int_y_list = start_pixel[1] + (y_diff * self.int_y_exp_vals)
            
            int_check_x_list = start_pixel[0] + (x_diff * self.int_check_points)
            int_check_y_list = start_pixel[1] + (y_diff * self.int_check_points)
            
            # Check if any of the intermediate pairs land in the obstacle or clearance
            for int_x, int_y in zip(int_check_x_list, int_check_y_list):
                if list(self.world_map[int(int_x), int(int_y)]) == self.map_colors["obstacle"] or list(self.world_map[int(int_x), int(int_y)]) == self.map_colors["clearance"]:
                    valid_line = False
                    break
                
            if valid_line == False:
                continue
            
            # Create the new configuration based on step size and the new angle
            new_loc = (new_x, new_y, new_angle)
            # print(start_pixel[0], start_pixel[1], start_pixel[2])
            # print(new_loc)
            # print()

            if new_loc[0] >= self.map_dim[0] or new_loc[0] < 0:
                continue

            if new_loc[1] >= self.map_dim[1] or new_loc[1] < 0:
                continue
            
            # Check if we're in an obstacle or clearance pixel
            if list(self.world_map[new_loc[0], new_loc[1]]) == self.map_colors["obstacle"] or list(self.world_map[new_loc[0], new_loc[1]]) == self.map_colors["clearance"]:
                # If we are don't add it to the list of new nodes
                # print("HIT OBSTACLE")
                continue
            
            # Calculate cost to get to the new pixel from the parent pixel as the euclidian distance between the 2
            cost_to_come = cost + math.sqrt((start_pixel[0] - new_loc[0])**2 + (start_pixel[1] - new_loc[1])**2)

            # Calculate the cost to come as the euclidian distance between the ne config and the goal config
            cost_to_go = math.sqrt((self.goal_node[0] - new_loc[0])**2 + (self.goal_node[1] - new_loc[1])**2)
            
            # print("Cost to come: {}".format(cost_to_come))
            # print("Cost to go: {}".format(cost_to_go))

            total_cost = self.c2g_weight*cost_to_go + cost_to_come 
            
            # Check if the configuration is in the list of working configs and grab it's current cost if it is
            if str(new_loc) in self.checked_pixels:
                existing_node = self.checked_nodes[str(new_loc)]
                
                existing_cost = existing_node[0]
                
                # If the new found cost is less than the existing cost then update the node with the new cost and 
                # parent pixel that gives it the lower cost
                if total_cost < existing_cost:
                    updated_node = (total_cost, existing_node[1], start_pixel, existing_node[3], cost_to_come, [x_vel, y_vel, ang_vel])
                    
                    # Update the checked nodes dict with the updated now
                    self.checked_nodes[str(new_loc)] = updated_node
                    
                    # Add the node with the updated cost to the priority queue
                    self.open_list.put(updated_node)
                    
                 
            # Otherwise add the new pixel to the checked nodes and pixel locations we're tracking   
            else:
                new_node = (total_cost, self.node_index, start_pixel, new_loc, cost_to_come, [x_vel, y_vel, ang_vel])
                self.node_index += 1
                
                # Update the drawing map with the latest explored node
                if self.use_lines:
                    # cv2.line(self.draw_map, (start_pixel[1], start_pixel[0]), (new_loc[1], new_loc[0]), color=self.map_colors["explored"], thickness=4)
                    curve = np.column_stack((np.array(int_y_list), np.array(int_x_list)))
                    cv2.polylines(self.draw_map, np.int32([curve]), False, self.map_colors["explored"], thickness=4)
                else:
                    self.draw_map[new_loc[0], new_loc[1]] = self.map_colors["explored"]  
                
                # Write the latest frame to the video
                if self.record and self.node_index % self.save_every_n_frames == 0:
                    # Make the maps smaller before writing
                    write_map = cv2.resize(self.draw_map, (int(self.map_dim[1]/self.save_scale), int(self.map_dim[0]/self.save_scale)))
                    self.video_rec.write(write_map)  
                
                # Update the checked nodes dict with the updated now
                self.checked_nodes[str(new_loc)] = new_node
                self.checked_pixels.add(str(new_loc))
                
                # Add the new node to the priority queue
                self.open_list.put(new_node)
                
            # cv2.imshow("Map", self.draw_map)
            # cv2.waitKey(0)
        
    # backtrack from the goal node to trace a path of pixels 
    def backtrack(self, goal_node):
        pix_loc = goal_node[3]
        move = goal_node[5]

        
        self.path_pixels.append(pix_loc)
        self.path_commands.append(move)
        
        # Loop through the parent nodes until we get back to the start location
        while not pix_loc == self.start_node:
            # Draw the path pixel
            self.draw_map[pix_loc[0], pix_loc[1]] = self.map_colors["path"]
            
            # Grab the previous node from it's pixel
            prev_node = self.checked_nodes[str(pix_loc)]
            
            # Grab the next pixel from the current node's parent pixel
            pix_loc = prev_node[2]
            move = prev_node[5]
            
            # print(pix_loc)
            
            # Save the pixel so we can trace it later
            self.path_pixels.append(pix_loc)
            self.path_commands.append(move)
            
        # Animate the path
        self.path_pixels.reverse()
        self.path_commands.reverse()
        
        for idx, pixel in enumerate(self.path_pixels):
            if self.use_lines:
                # Check if it's the final node in the path
                if idx == len(self.path_pixels) - 1:
                    # print("Final path node reached")
                    self.draw_map = cv2.rectangle(self.draw_map, 
                                        (pixel[1], pixel[0]), 
                                        (pixel[1] + 20, pixel[0] + 20),
                                        color=self.map_colors["path"],
                                        thickness=-1)
                else:
                    # print("Drawing path")
                    next_pix = self.path_pixels[idx+1]
                    # x_diff = next_pix[0] - pixel[0]
                    # y_diff = next_pix[1] - pixel[1]
                    
                    # int_x_list = pixel[0] + (x_diff * self.int_x_exp_vals)
                    # int_y_list = pixel[1] + (y_diff * self.int_y_exp_vals)
                    
                    # curve = np.column_stack((np.array(int_y_list), np.array(int_x_list)))
                    # cv2.polylines(self.draw_map, np.int32([curve]), False, self.map_colors["path"], thickness=4)
                    self.draw_map = cv2.line(self.draw_map, (pixel[1], pixel[0]), (next_pix[1], next_pix[0]), color=self.map_colors["start"], thickness=8)
                    
            else:
                self.draw_map = cv2.rectangle(self.draw_map, 
                                        (pixel[1], pixel[0]), 
                                        (pixel[1] + 20, pixel[0] + 20),
                                        color=self.map_colors["path"],
                                        thickness=-1)
            if self.record:
                # Make the maps smaller before writing
                write_map = cv2.resize(self.draw_map, (int(self.map_dim[1]/self.save_scale), int(self.map_dim[0]/self.save_scale)))
                self.video_rec.write(write_map) 
            
            
        # Draw the start and goal nodes again
        self.draw_map = cv2.rectangle(self.draw_map, 
                                      (self.start_node[1], self.start_node[0]), 
                                      (self.start_node[1] + 20, self.start_node[0] + 20),
                                      color=self.map_colors["start"],
                                      thickness=-1)
        
        self.draw_map = cv2.rectangle(self.draw_map, 
                                      (self.goal_node[1], self.goal_node[0]), 
                                      (self.goal_node[1] + 20, self.goal_node[0] + 20),
                                      color=self.map_colors["goal"],
                                      thickness=-1)
        
        # Write the final frame to the video
        if self.record:
            # Write the final frame multiple times to let it show for a while
            for i in range(300):
                # Make the maps smaller before writing
                write_map = cv2.resize(self.draw_map, (int(self.map_dim[1]/self.save_scale), int(self.map_dim[0]/self.save_scale)))
                self.video_rec.write(write_map) 
            
            
    # Checks if the passed in node is within the acceptable threshold of the goal
    def checkIfGoal(self, input_node):
        node = input_node[3]
        # print("Distance to goal: {}".format(math.sqrt((self.goal_node[0] - node[0])**2 + ((self.goal_node[1] - node[1])**2))))
 
        # Check euclidian distance between x and y
        if not math.sqrt((self.goal_node[0] - node[0])**2 + (self.goal_node[1] - node[1])**2) <= self.dist_tolerance:
            return False
        # Check difference between the current and goal angle
        elif not abs(self.goal_node[2] - node[2]) <= self.angle_tolerance:
            return False
        else:
            return True
    
    # Finds the shortest path from the start to end goal using Dijkstra's algorithm
    def findPath(self):
        start_time = time.time()
        # Loop until we've removed all nodes from the open list
        while not self.open_list.empty():
            # Pop off the highest priority node
            priority_node = self.open_list.get()
            
            # print(priority_node)
            
            # First check if this is the goal node
            if self.checkIfGoal(priority_node):
                print("Goal found")
                # If it is then backtrack to the start node and break out of the loop
                self.backtrack(priority_node)
                self.goal_found = True
                break
            
            cost = priority_node[4]
            
            pixel_loc = priority_node[3]
            
            self.applyMoves(pixel_loc, cost)
            
            # Move the current node to the closed nodes list
            self.closed_list[str(pixel_loc)] = priority_node
            
        if not self.goal_found:
            print("UNABLE TO FIND PATH TO GOAL FROM GIVEN CONFIGURATIONS. PLEASE ENTER A NEW CONFIGURATION.")

        print("Total time: {} seconds".format(time.time() - start_time))

        if self.record:
            self.video_rec.release()
        
        show_map = cv2.resize(self.draw_map, (int(self.map_dim[1]), int(self.map_dim[0])))
        cv2.imshow("Exploration map", show_map)
        cv2.waitKey(0)

        # Loop through the saved path commands and execute each one of them
        # TODO: Replace this with the custom path pub
        # for command in self.path_commands:
        #     # Extract the individual velocity components
        #     # Convert from mm to m
        #     x_vel = -command[0]/(14000)
        #     y_vel = command[1]/(14000)
        #     ang_vel = -self.deg2rad(command[2]*7)

        #     # print(command)

        #     total_vel = math.sqrt(x_vel**2 + y_vel**2)

        #     # Create a blank message and populate it
        #     twist_msg = Twist()

        #     twist_msg.linear.x = total_vel
        #     # twist_msg.linear.y = y_vel
        #     twist_msg.angular.z = ang_vel

        #     # print(twist_msg)

        #     print("(X Vel: {}, Ang Vel: {})".format(total_vel, ang_vel))

        #     self.vel_pub.publish(twist_msg)

        #     time.sleep(self.timestep*110)

        # # Create a final message with 0 vel and publish it to stop the robot
        # twist_msg = Twist()

        # twist_msg.linear.x = 0.0
        # twist_msg.angular.z = 0.0

        # self.vel_pub.publish(twist_msg)

def main(args=None):
    # rclpy.init(args=args)
    # node = MapSolver()
    # try:
    #     node.pubPath()
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()

    rclpy.init(args=args)
    node = AStarMapSolver(record_video=True, c2g_weight=2, use_lines=True, save_every_n_frames=100)
    try:
        node.findPath()
    finally:
        node.destroy_node()
        rclpy.shutdown()      

if __name__ == "__main__":
    main()
