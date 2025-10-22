
--Terminal 1 --
ros - to start apptainer
roscore

--Terminal 2 --
rosrun turtlesim turtlesim_node

--Terminal 3 --
Package is already made (turtle_rectangle_pkg)
rosrun package python file.py


--How to make Package--
Apptainer> mkdir -p ~/catkin_ws/src
Apptainer> cd ~/catkin_ws/src
Apptainer> catkin_create_pkg turtle_rectangle_pkg rospy geometry_msgs turtlesim
Apptainer> mkdir ~/catkin_ws/src/turtle_rectangle_pkg/scripts
Apptainer> mv /path/to/task1_node.py ~/catkin_ws/src/turtle_rectangle_pkg/scripts/
Apptainer> mv /path/to/turtle_rectangle.py ~/catkin_ws/src/turtle_rectangle_pkg/scripts/
Apptainer> chmod +x ~/catkin_ws/src/turtle_rectangle_pkg/scripts/*.py
Apptainer> cd ~/catkin_ws
Apptainer> catkin_make
Apptainer> source devel/setup.bash

-- Creates Package and Workspace --
