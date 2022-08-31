# Autonomous-Wheelchair
A project that was done for a robotics course. Developed a feedback controller for a wheelchair with differential drive. Implemented Markov localization for the wheelchair to find its location on a given map. Implemented path planning and obstacle avoidance using the A* algorithm.

The project consists of a single MATLAB file called "Group1_ME574_Robot_Project". 
The map.png file is the map of the environment, which is done to mimic a disorganized home.
Detailed information is given in the ME574_PROJECT_REPORT file.

The progress in the code is as this:
Given a map environment, the robot tries to localize itself with Markov localization to find its location.
After localization is done, a goal position is given. Then, using A* algorithm, a 2D matrix of waypoints between the robot's position and goal position is created.
The robot uses proportional controller to reach these waypoints while being subject to differential drive kinematics constraints.
