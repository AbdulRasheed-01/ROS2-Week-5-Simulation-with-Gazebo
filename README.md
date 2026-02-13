# ROS2-Week-5-Simulation-with-Gazebo

🎯 Learning Objectives
By the end of this week, you will be able to:

✅ Understand Gazebo simulation architecture

✅ Create URDF and SDF robot models

✅ Simulate robots in complex environments

✅ Add sensors (cameras, LiDAR, IMU) to robots

✅ Implement ROS 2 control with ros2_control

✅ Create custom worlds and plugins

✅ Simulate multiple robots simultaneously

✅ Integrate Gazebo with MoveIt 2 and Navigation2

📚 Theory Content

5.1 What is Gazebo?

Gazebo is a 3D robotics simulator with:

Physics engine (ODE, Bullet, DART, Simbody)

Sensor simulation (cameras, LiDAR, IMU, GPS)

3D graphics (Ogre)

Programmatic interface (plugins)

ROS 2 integration (ros_ign_bridge)

Gazebo vs Other Simulators:

Feature	      |  Gazebo Classic	 |   Gazebo Ignition	  |   Webots	 |  CoppeliaSim

ROS 2 Support	|  Legacy	         |   Native	            |   Good	   |  Good

Physics	      |  ODE/Bullet	     |   DART	              |   ODE	     |  Bullet

Graphics	    |  OGRE	           |   OGRE 2	            |   Custom	 |  Custom

Performance	  |  Good            |   Excellent	        |   Good	   |  Steep

Gazebo Fortress (Current) Features:

Distributed simulation

Better performance

Native ROS 2 integration

Improved rendering

Python API

5.2 URDF vs SDF

URDF (Unified Robot Description Format):

XML format for robot descriptions

ROS-specific

Limited to robots only

No world description

Kinematics/Dynamics only

SDF (Simulation Description Format):

XML format for entire simulation

Gazebo-native

Robots, worlds, lights, physics

Complete simulation description

More expressive

When to use which:

URDF: Robot description for ROS 2 (MoveIt, Navigation)

SDF: Full simulation description (Gazebo)

Xacro: Macro language for URDF (parametric robots)

Key Components:

Controller Manager: Loads/unloads controllers

Joint Controllers: Position, velocity, effort control

Hardware Interface: Abstract robot hardware

Gazebo Plugin: Bridge between ROS 2 and Gazebo

5.4 Sensor Simulation

Supported Sensors in Gazebo:

Sensor	        |              Topic Type	             |       Use Case

Camera	        |              sensor_msgs/Image	     |         Vision, object detection

Depth Camera	  |              sensor_msgs/Image	     |        3D perception

LaserScan	      |              sensor_msgs/LaserScan	 |      2D LiDAR

PointCloud	    |              sensor_msgs/PointCloud2 |	    3D LiDAR

IMU	sensor_msgs/Imu	 |         Orientation,            |    acceleration

GPS	            |              sensor_msgs/NavSatFix	 |    Global position

Force/Torque	  |              geometry_msgs/Wrench	   |   Contact sensing

Sonar	          |              sensor_msgs/Range	     |    Underwater/ultrasonic

