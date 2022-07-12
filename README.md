# Sports Field Security Guard (formerly the RoomTidierUR3)
This is our project for ECE470: Robot Kinematics

We assume the user is on Linux Ubuntu and using Gazebo to simulate. We based our project on the UR3 library of functions written for RoS. Credit for the tutorials used goes to Universal Robotics https://www.universal-robots.com/products/ur3-robot/

Current version:

  The robot will stand at the sideline of a sports field and detect when someone has entered the field that are not players. In reality, this could be done using a full-field camera that simply counts the amount of players and removes the most recent entry. Our robot is able to remove unwanted spectators anywhere from within the workspace and place them behind the line. In the future, we hope to add a track that will allow the robot to patrol the entire perimeter. 

How to operate the project (Ubuntu)


1st terminal: 

  `roslaunch ur3_driver ur3_gazebo.launch`
  
2nd terminal: 

  `rosrun lab2pkg_py lab2_spawn.py`
  
  `rosrun lab2pkg_py lab2_exec.py`
  
Camera feed:

  `rosrun image_view image_view image:=/cv_camera_node/image_raw`
