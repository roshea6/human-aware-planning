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
from sklearn.neighbors import NearestNeighbors
import random

class AStarMapSolver(Node):
    def __init__(self, record_video=False, c2g_weight=1, use_lines=False, save_every_n_frames=500, use_prm=False):
        super().__init__('a_star_navigation_node')

        # self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.path_pub = self.create_publisher(Path, 'custom_path', 10)

        # Define the map colors
        self.map_colors = {"obstacle": [0, 0, 127], # Dark Red
                           "clearance": [0, 255, 0], # Green
                           "unexplored": [0, 0, 0], # Black
                           "explored": [255, 0, 0], # Blue
                           "path": [255, 255, 255], # White
                           "start": [0, 255, 255], # Yellow
                           "goal": [255, 0, 255], # Purple
                           "human_obstacle": [0, 0, 255], # Bright Red
                           "human_space": [0, 165, 255], # Orange
                           "human_view": [203, 192, 255] # pink 
                        }  
        
        # HUMAN AWARE PARAMS
        self.human_comfort_rad = 80
        self.human_vision_rad = 40
        self.human_vis_offset = 70
        self.min_human_comfort_scale = 1.0
        self.max_human_comfort_scale = 1.02
        self.min_human_vision_scale = 1.0
        self.max_human_vision_scale = 1.01
        self.num_cost_rings = 5
        self.human_fov = 120

        self.use_prm = use_prm

        
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
        # while True:
        #     self.clearance = int(input("Enter the clearance value in mm (0-10 recommended): "))
            
        #     if self.clearance < 0 or self.clearance > 10:
        #         continue
        #     else:
        #         break

        self.clearance = 5
            
        # Get the two rpms from the user
        # while True:
        #     self.rpm1 = int(input("Enter the first rpm (30-70 recommended): "))
            
        #     if self.rpm1 < 0 or self.rpm1> 100:
        #         continue
        #     else:
        #         break
        # while True:
        #     self.rpm2 = int(input("Enter the second rpm (30-70 recommended): "))
            
        #     if self.rpm2 < 0 or self.rpm2> 100:
        #         continue
        #     else:
        #         break

        self.rpm1 = 30
        self.rpm2 = 70
            
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
        
        # Make a cost map of 1s to represent multipliers
        self.human_prox_cost_map = np.ones((self.map_dim[0], self.map_dim[1]), np.uint8)
        self.human_vis_cost_map = np.ones((self.map_dim[0], self.map_dim[1]), np.uint8)

        # cv2.imshow("Blank", blank_map)
        # cv2.waitKey(0)
        
        # Person 1
        top_left = (390, 290)
        bottom_right = (410, 310)
        center = (int((top_left[0] + bottom_right[0])/2), int((top_left[1] + bottom_right[1])/2))
        
        # Add in human vision half circle
        axes = (self.human_vision_rad, self.human_vision_rad)
        angle = -30
        startAngle = 0
        endAngle = (360-self.human_fov)

        # obstacle_map = cv2.ellipse(blank_map, center, axes, angle, startAngle, endAngle, color=self.map_colors["human_view"], thickness=-1)
        # Draw the human comfort circle
        obstacle_map = cv2.circle(blank_map, 
                                  center,
                                  self.human_comfort_rad,
                                  color=self.map_colors["human_space"],
                                  thickness=-1)

        # Draw the human vision circle
        obstacle_map = cv2.circle(obstacle_map, 
                                  (center[0], center[1] + self.human_vis_offset),
                                  self.human_vision_rad,
                                  color=self.map_colors["human_view"],
                                  thickness=-1)
        
        # Draw the concentric human comfort cost circles on the proximity cost map
        ring_rads = np.linspace(0, self.human_comfort_rad, self.num_cost_rings + 1)
        
        # Convert to ints, drop the first one, and reverse so we start with the biggest ring
        ring_rads = [int(rad) for rad in ring_rads][1:]
        ring_rads.reverse()

        # Calculates the cost step size increase between rings
        self.human_prox_cost_inc = (self.max_human_comfort_scale - self.min_human_comfort_scale)/self.num_cost_rings
        self.human_vis_cost_inc = (self.max_human_vision_scale - self.min_human_vision_scale)/self.num_cost_rings

        # Draw the concentric human comfort cost circles on the proximity cost map
        vis_ring_rads = np.linspace(0, self.human_vision_rad, self.num_cost_rings + 1)
        
        # Convert to ints, drop the first one, and reverse so we start with the biggest ring
        vis_ring_rads = [int(rad) for rad in vis_ring_rads][1:]
        vis_ring_rads.reverse()

        # Add human prox cost and vis cost rings for first person
        for idx, (prox_rad, vis_rad) in enumerate(zip(ring_rads, vis_ring_rads)):
            cv2.circle(self.human_prox_cost_map, 
                        center,
                        prox_rad,
                        color=(idx+1)*25,
                        thickness=-1)
            
            cv2.circle(self.human_vis_cost_map, 
                        (center[0], center[1] + self.human_vis_offset),
                        vis_rad,
                        color=(idx+1)*25,
                        thickness=-1)
        

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
                                      color=self.map_colors["human_obstacle"])
        

        # Person 2
        top_left = (590, 190)
        bottom_right = (610, 210)
        center = (int((top_left[0] + bottom_right[0])/2), int((top_left[1] + bottom_right[1])/2))

        axes = (self.human_vision_rad, self.human_vision_rad)
        angle = 60
        startAngle = 0
        endAngle = (360-self.human_fov)
        # cv2.ellipse(obstacle_map, center, axes, angle, startAngle, endAngle, color=self.map_colors["human_view"], thickness=-1)
        # Draw the human comfort circle
        obstacle_map = cv2.circle(obstacle_map, 
                                  center,
                                  self.human_comfort_rad,
                                  color=self.map_colors["human_space"],
                                  thickness=-1)

        # Draw the human vision circle
        obstacle_map = cv2.circle(obstacle_map, 
                                  (center[0] - self.human_vis_offset, center[1]),
                                  self.human_vision_rad,
                                  color=self.map_colors["human_view"],
                                  thickness=-1)
        
        # Add human prox cost rings for second person
        for idx, (prox_rad, vis_rad) in enumerate(zip(ring_rads, vis_ring_rads)):
            cv2.circle(self.human_prox_cost_map, 
                        center,
                        prox_rad,
                        color=(idx+1)*25,
                        thickness=-1)
            
            cv2.circle(self.human_vis_cost_map, 
                        (center[0] - self.human_vis_offset, center[1]),
                        vis_rad,
                        color=(idx+1)*25,
                        thickness=-1)
        
        # cv2.imshow("Human Proxity Cost Field", self.human_prox_cost_map)
        # cv2.imshow("Human Vision Cost Field", self.human_vis_cost_map)
        # cv2.waitKey(0)
        # cv2.imwrite("./src/path_pub_package/output_imgs/Human_proximity_cost_field.png", self.human_prox_cost_map)
        # cv2.imwrite("./src/path_pub_package/output_imgs/Human_vision_cost_field.png", self.human_vis_cost_map)
        # exit()


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
                                      color=self.map_colors["human_obstacle"])
        
        # Border edges
        # Left wall
        obstacle_map = cv2.rectangle(obstacle_map, 
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
        
        # Generate n accpetable nodes and connect them after clusting them
        if self.use_prm:
            self.prm_array = set()
            for i in range(3000):
                x = random.randint(30, self.map_dim[0] - 30)
                y = random.randint(30, self.map_dim[1] - 30)

                x = int(self.search_loc_thresh * round(x/(self.search_loc_thresh)))
                y = int(self.search_loc_thresh * round(y/(self.search_loc_thresh)))

                if list(obstacle_map[x, y]) == self.map_colors["obstacle"] or list([x, y]) == self.map_colors["clearance"]:
                    continue

                else:
                    self.prm_array.add(str([y, x]))

                obstacle_map = cv2.circle(obstacle_map, 
                    (y, x),
                    4,
                    color=(255, 255, 255),
                    thickness=-1)

        # cv2.imshow("Map", obstacle_map)
        # while not cv2.waitKey(0) == ord("q"):
        #     print("nope")

        # exit()
        
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

        # while True:
        #     self.step_size = int(input("Please enter a step size value between 1 and 3: "))

        #     if self.step_size < 1 or self.step_size > 3:
        #         print("Invalid step size")
        #     else:
        #         # Multiply step by 2 to account for the larger map
        #         self.step_size *= 2
        #         break

        self.step_size = 4
            
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

            # print(str([new_y, new_x]))

            # Check if the new pixel is in the pregenerated roadmap graph and skip if it's not
            if self.use_prm:
                if not str([new_y, new_x]) in self.prm_array:
                    continue

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

            # Multiply the cost to go value by the human cost scaling factors
            human_prox_scaling = 1 + ((self.human_prox_cost_map[new_loc[0]][new_loc[1]]-1) * self.human_prox_cost_inc)
            human_vis_scaling = 1 + ((self.human_vis_cost_map[new_loc[0]][new_loc[1]]-1) * self.human_vis_cost_inc)

            cost_to_come *= human_prox_scaling*human_vis_scaling

            # print(cost_to_go)
            
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
            
            # total_cost = prev_node[0]
            # c2c = prev_node[4]

            # print("Cost 2 Come: {}".format(c2c))
            # print("Total cost: {}".format(total_cost))
            
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

        # while not cv2.waitKey(0) == ord("q"):
        #     print("nope")

        # Create blank path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        waypoints = []

        for idx, (pix_y, pix_x, ang) in enumerate(self.path_pixels):
            # Convert the pixel coordinates to real world coordinates
            x = pix_x/100
            y = (self.map_dim[0] - pix_y)/100 # Need to subtract from the map y because opencv's coorinate system starts in the upper left


            # TODO: Something is messed up in either the angle calculation or the quaternion calculation
            roll = 0
            pitch = 0
            if idx == 0:
                yaw = ang
            else:
                prev_y = self.path_pixels[idx - 1][0]
                prex_x = self.path_pixels[idx - 1][1]

                yaw = math.atan2((y-prev_y), (x-prex_x))


            # Convert to quaternion
            q_x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            q_y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            q_z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            q_w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

            pose_stamped = PoseStamped()
            # pose_stamped.header.stamp = self.get_clock().now().to_msg()
            # pose_stamped.header.frame_id = 'map'  # Set the frame ID
            pose_stamped.pose.position.x = x  # Set x-coordinate
            pose_stamped.pose.position.y = y  # Set y-coordinate
            pose_stamped.pose.position.z = 0.0   # Set z-coordinate

            # Populate orientation part of message
            pose_stamped.pose.orientation.x = q_x
            pose_stamped.pose.orientation.y = q_y
            pose_stamped.pose.orientation.z = q_z
            pose_stamped.pose.orientation.w = q_w

            waypoints.append(pose_stamped)
        # Populate the Path message with the waypoints
        path_msg.poses = waypoints

        print("Publishing")
        # print(path_msg)

        # Publish the Path message
        self.path_pub.publish(path_msg)

        

def main(args=None):
    # rclpy.init(args=args)
    # node = MapSolver()
    # try:
    #     node.pubPath()
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()

    rclpy.init(args=args)
    node = AStarMapSolver(record_video=False, c2g_weight=2, use_lines=True, save_every_n_frames=10, use_prm=True)
    try:
        node.findPath()
    finally:
        node.destroy_node()
        rclpy.shutdown()      

if __name__ == "__main__":
    main()
