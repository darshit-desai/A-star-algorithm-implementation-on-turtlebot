# A-star-algorithm-implementation-on-turtlebot
An implementation of A* on turtlebot which is than simulated in ROS and Gazebo.
*Project 3 Phase 2 ENPM661 Path Planning for Autonomous Robots*
## Authors
- [Darshit Desai](https://github.com/darshit-desai); Dir ID: darshit; UID: 118551722
- [Shivam Sehgal](https://github.com/shivamsehgal77); Dir ID: ssehgal7; UID: 119222041
## Code run Instructions

To run the program ensure that you have the following libraries: time, pygame, numpy, queue

Download the code using https://github.com/darshit-desai/A-star-algorithm-implementation-on-turtlebot

### For Part 1

For running the code use /bin/python3 /$PATH$/a_star_DarshitMiteshkumar_Shivam.py in the linux or VSCode Terminal

The example set of inputs are given below:

        ROBOT CLEARANCE DIMENSIONS AND RADIUS(Radius is fixed). Enter valid dimensions between 0 to 50
        Enter the clearance of the robot: 5
        Valid coordinates received
        Enter Robot start and goal coordinates. Ensure that theta values are multiples of 30 deg
        Enter the starting x coordinate: 25
        Enter the starting y coordinate: 25
        Enter the start theta orientation: 0
        Enter the goal x coordinate:500
        Enter the goal y coordinate: 110
        Enter the goal theta orientation: 0
        Enter the Robot RPM sets
        Enter Robot wheel RPM 1: 5
        Enter Robot wheel RPM 2: 10

### For Part 2

For this the robot start position has been hard coded and set to (-25,-75), subsequently the values of rates and RPMs were tuned to get the desired output, One of the combinations of that output is mentioned below: (Note the coordinates inserted in the terminal are from the coordinate system considered in the bottom left corner of the pygame screen)

    ROBOT CLEARANCE DIMENSIONS AND RADIUS(Radius is fixed). Enter valid dimensions between 0 to 50
    Enter the clearance of the robot: 5
    Valid coordinates received
    Enter Robot start and goal coordinates. Ensure that theta values are multiples of 30 deg
    Enter the starting x coordinate: 25
    Enter the starting y coordinate: 25
    Enter the start theta orientation: 0
    Enter the goal x coordinate:500
    Enter the goal y coordinate: 100
    Enter the goal theta orientation:0
    Enter the Robot RPM sets
    Enter Robot wheel RPM 1: 15
    Enter Robot wheel RPM 2: 20
    
  * To run the ros node, Download the package from the below command:
 
      git clone https://github.com/darshit-desai/A-star-algorithm-implementation-on-turtlebot
      
  * After this copy and paste the package in your catkin workspace  

        cd ~/catkin_ws
        catkin_make
        source devel/setup.bash
        roslaunch a_starTB3 astar.launch
        rosrun a_starTB3 astar.py

  * Enter the inputs in the terminal, The robot should now moving towards the goal. To close gazebo, Press Ctrl+C

## Results:

The results are currently stored in the google drive where phase 1 and phase 2 results are simulated together in two videos as mentioned in the rubric

  * ![Video with goal point after the circle solution](https://drive.google.com/file/d/1EDF_HijzBKSzLHSFyeexxuSAhnIsqLh-/view?usp=share_link)
  * ![Video with goal point after the rectangles solution](https://drive.google.com/file/d/11D_dVWgj3gcSE6QI9WmnFu7JgLugiG3B/view?usp=share_link)

## Dependencies
    * Python 3.8
    * pygame
    * ROS Noetic
    * Priority Queue
