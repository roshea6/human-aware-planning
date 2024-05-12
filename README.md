# Ryan O'Shea and Kyle DeGuzman ENPM661 Project 5: Human Aware Motion Planner For The Turtlebot3

# Contributors
- Ryan O'Shea: roshea, 120465291
- Kyle DeGuzman: kdeguzma, 120452062

# Project Link
https://github.com/roshea6/human-aware-planning

# Dependencies
Python
- numpy
- copy
- opencv-python
- queue
- math
- time
- queue

ROS via github. These pacakges will all need to be cloned into the workspace if not already there.
- turtlebot3 packages (https://github.com/ROBOTIS-GIT/turtlebot3 galactic-devel branch)
- turtlebot3_simulations packages (https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git galactic-devel branch)
- turtlebot3_msgs packages (https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git galactic-devel branch)

ROS via apt-get
- ros-galactic-navigation2
- ros-galactic-nav2-bringup


# Pure Path Planning Running Instructions
- The path_pub.py file can be run as is to take in a start and goal state (x, y, theta) and find the shortest path between them. By defaul this will display the end map and will publish the path after exiting from the pop up map.
- To turn on the video creation function, set the record_video variable on line 796 to true.
- The x and y values for the start and end points are input by the user upon running the program. The following pairs were used to produced the output shown in the video:
    - Enter start pixel x value: 100 (For the first run with a sim trial the start x, y should always be 100, 100 because that's where the turtlebot spawn's in the sim)
    - Enter start pixel y value: 100
    - Enter start theta value that is a multiple of 30: 0
    - Enter goal pixel x value: 700
    - Enter goal pixel y value: 300
    - Enter start theta value that is a multiple of 30: 0
- This should produce a path that starts in the lower left of the image and ends in the top right while navigating around obstacles
- The path taken will be dependent on the values of the human awareness parameters
- The colors shown on the map have the following meaning
    - Green: clearance around obstacles
    - Dark Red: Static Obstacles
    - Light Red: Human obstacle
    - Orange: Human proximty space
    - Pink: Human no vision zone
    - Black: Unexplored space
    - Blue: Explored spaces
    - Yellow Square: Start point and traced path
    - Purple: Goal point

- Other good goals:
    - Enter goal pixel x value: 650
    - Enter goal pixel y value: 100
    - Enter goal pixel x value: 400
    - Enter goal pixel y value: 200

- Human Aware Parameters: Lines 37 - 44 have 8 different human awareness parameters with the following effects:
    - self.human_comfort_rad = 100: Radius in pixels of the human comfort area. 100 pixels = 1m
    - self.human_vision_rad = 60 Radius in pixels of human no vision circle
    - self.human_vis_offset = 70 How far behind the location of the human the no vision circle should start
    - self.min_human_comfort_scale = 1.0 Minimum value at edge of human proximity comfort
    - self.max_human_comfort_scale = 1.2 Maximum scaling value for final inner ring of human proximity (No human aware = 1.0, low = 1.02, high = 1.1)
    - self.min_human_vision_scale = 1.0 Minimum value at edge of human vision comfort
    - self.max_human_vision_scale = 1.1 Maximum scaling value for final inner ring of human vision (No human aware = 1.0, low = 1.01, high = 1.05)
    - self.num_cost_rings = 5 # Number of discrete costs zones in the human comfort areas: Higher number split up the space further and make a smoother transition between zones. This can also lead to more navigation through the outer rings because their cost scaling will be lower unless the comfort scales are increased accordingly

- Additional inputs: Line 796 also allows you to enter several other inputs for the path planner to change its behavior
    - c2g_weight: Scaling factor for the cost to go in the A* algorithm. Values above 1 will make it weighted A*, 1 excatly will make it standard A* and 0 will make it Dijkstra's
    - use_lines: Whether to draw lines between start and end points during explortation instead of just filling in individual explored pixels
    - save_every_n_frame: How many frames to skip between saving a frame during video recording. Wihtout this, saving every video frame greatly slows down the code and make a giant video.

# Full Simulation Instructions
- To execute the path on a simulated turtlebot3 several steps need to be taken
- First run the path_pub.py code as normal either as a python file or ROS node but don't exit out of the generated exploration map
    - This can technically be done after launching the simulation but it's much faster this way because Gazebo+RVIZ bog down my laptop immensely
- Launch the following two launch files to run the gazebo and rviz simulations respectively
    - ros2 launch nav2_human_aware_planner turtlebot3_human_world.launch.py
    - ros2 launch nav2_human_aware_planner human_navigation2.launch.py 
- Once Gazebo and RVIZ2 are both launched, use the RVIZ 2D pose estimate tool to place a pose estimate on 1, 1 with the arrow facing towards the opposite side of the map. This will bring up the static costmaps for the local planner to use
- Then use the Nav2 goal to set a fake goal anywhere on the map. This goal doesn't actually do anything besides send a request to the global planner to make a path. However, the global planner is waiting on the path produced by path_pub.py
- Finally, exit out of the opencv displayed exploration map to publish the path to the simulation. The path should appear in RVIZ and the turtlebot should begin to execute it
- The full launch process can be seen at the following link to help clear things up: https://drive.google.com/file/d/1UKoOVQ77dqVNJwLFj8jWNvJf4btwjYQh/view?usp=sharing 
