# RBE3002 Final Project 

Joseph Lombardi, Brandon Kelly, William Godsey

Code Organization:
The code is organized into imports, initial defines, two classes, and a main function. Starting at the top and moving downwards, the first thing is the general import for methods and message types we used throughout the program followed by the initially defined value of PI and the Z axis.  The Next segment is the node class, which is a class with defined variables in order to hold its values for the following atributes: x coordinate, y coordinate, cost, frontier number, row, and collumn. Following the Node class is the FinalProject class which contains all of the variables and methods required in order to store all of the data needed and perform all the neccessary action to complete the task.  Finally, at the bottom of the script is the main function. IT defines the node and calls the defined functionality in order for the robot to perform the exploration of a space.

How to Run the Program:
1. Place the robot in region to be mapped
2. Bringup the turtlebot:
	> roslaunch turtlebot_bringup minimal.launch --screen
3. Launch Gmapping:
	> roslaunch turtlebot_navigation gmapping_demo.launch
4. Launch RVIZ:
	> roslaunch turtlebot_rviz_launchers view_navigation.launch
5. To visualize frontiers and largest group, add two GridCell visualizations to RVIZ mapped on respective topics,
	>/frontier
	
	>/theFrontier
6. Run the final project python file:
	> cd <file_location>
	
	>python theLastRobot.py
	
