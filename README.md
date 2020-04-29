# Obstacle-Avoidance-Bot-Using-ROS
This project contains a bot simulation in Gazebo using bug0 algorithm. The robot design and other requirements can be found in this [PDF](assignment.pdf)
In this program the bot reaches the given goal by following the boundary of the obstacle if any. All codes are written in python.
Xacro is used in the project to clean the URDF code.

## Getting Started

1. Clone this repository in the `src` folder of your `catkin` workspace
2. Run `catkin_make`
3. Go to the python codes in src folder and make it executable using `chmod +x <file-name>.py`. 
3. Open 3 Terminals
4. Run the command `roslaunch Path-planning-Algorithm-1 world1.launch`. 
5. In the second terminal run the command `roslaunch Path-planning-Algorithm-1 spawn.launch`.
6. In last terminal, run the command `rosrun Path-planning-Algorithm-1 bug0algo.py` to start the robot and begin using the bug0 algorithm. Replace bug0algo.py with wall_follow.py to run the wall-follow algorithm.
7. Add blocks in between wherever you want.

## Prerequisites

* [ROS](http://wiki.ros.org/kinetic)  
* [Gazebo](http://wiki.ros.org/gazebo_ros_pkgs)


## Video

Will upload the video soon.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
