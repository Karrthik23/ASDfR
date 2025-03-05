# ASDfR
Assignment for Advanced Software Development for Robotics

- Build the packages from workspace directory
- Change the IP-adress in src/Assignment1.1/cam2image_vm2ros/config/cam2image.yaml to host ip-address 

#############################
Assignment 1.1.2 
- Start videoserver on host and run cam2image package
- use: ros2 run brightness_checker brightness_checker
- Check command line to see brightness value
- Use: ros2 topic echo /brightness_status to listen to the status

#############################
Assignment 1.1.3
- To change value at launch use: ros2 run brightness_checker brightness_checker --ros-args -p brightness_threshold:=<value>
- To change during runtime use: ros2 param set /image_brightness_calculator brightness_threshold <value>

#############################
Assignment 1.1.4
- To run the ball detector package use: ros2 run ball_detector ball_detector 
- Check the command line to see pixel coordinates
- Subscribe to or echo the topic /pixel_coordinates to get data

#############################
Assignment 1.2.1
- To generate random setpoints change tracking_mode parameter to False in: src/Assignment1.1/cam2image_vm2ros/launch/assignment_launch.py
- To run all the nodes use: ros2 launch cam2image_vm2ros assignment_launch.py

#############################
Assignment 1.2.2
- To follow the detected object change tracking_mode parameter to True in: src/Assignment1.1/cam2image_vm2ros/launch/assignment_launch.py
- To run all the nodes use: ros2 launch cam2image_vm2ros assignment_launch.py 
