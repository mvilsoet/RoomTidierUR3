# RoomTidierUR3
This is our project for ECE470: Robot Kinematics
We assume the user is on Linux Ubuntu and using Gazebo to simulate. We based our project on the UR3 library of functions written for RoS. Credit for the tutorials used goes to Universal Robotics https://www.universal-robots.com/products/ur3-robot/

Current version:
  The robot will check for objects on the ground and will place them in a designated area. It has sensor feedback in the gripper that will cancel the grab in case it is unable to grab the object it planned to. It currently has a limited workspace; however, in the future, we hope to attach our arm to a Roomba-esque vehicle that will be able to move around the entire room and place objets into a designated bin. 

Useful commands:
:)
rosrun
