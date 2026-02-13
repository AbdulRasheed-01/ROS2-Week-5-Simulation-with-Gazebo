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

⚙️ Setup and Installation
Step 1: Install Gazebo Fortress

    #Add Gazebo repository
    sudo apt-get update
    sudo apt-get install lsb-release wget gnupg
    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg]   http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

    #Install Gazebo Fortress  
    sudo apt-get update
    sudo apt-get install ignition-fortress

    #Verify installation
    ign gazebo -v

Step 2: Install ROS 2 Gazebo Bridge

    #Install ros_gz (ROS 2 Gazebo bridge)
    sudo apt-get install ros-humble-ros-ign-bridge
    sudo apt-get install ros-humble-ros-ign-gazebo
    sudo apt-get install ros-humble-ros-ign-image

    #Install additional Gazebo ROS packages
    sudo apt-get install ros-humble-gazebo-ros-pkgs
    sudo apt-get install ros-humble-gazebo-ros2-control
    sudo apt-get install ros-humble-gazebo-ros2-control-demos

    #Install robot description packages
    sudo apt-get install ros-humble-robot-state-publisher
    sudo apt-get install ros-humble-joint-state-publisher
    sudo apt-get install ros-humble-joint-state-publisher-gui
    sudo apt-get install ros-humble-xacro

    #Install ros2_control
    sudo apt-get install ros-humble-ros2-control
    sudo apt-get install ros-humble-ros2-controllers
    sudo apt-get install ros-humble-control-toolbox

Step 3: Create Workspace Package

    cd ~/ros2_ws/src

    #Create simulation package
    ros2 pkg create gazebo_simulation --build-type ament_python \
        --dependencies rclpy rclcpp std_msgs sensor_msgs geometry_msgs \
                     gazebo_ros_pkgs gazebo_ros2_control xacro \
                     robot_state_publisher joint_state_publisher \
                     control_msgs ros2_control \
        --description "Week 5: Gazebo Simulation with ROS 2"

    cd gazebo_simulation

    #Create directory structure
    mkdir -p gazebo_simulation/{robots,worlds,sensors,controllers,launch}
    mkdir -p {urdf,meshes,config,models,worlds,launch}
    mkdir -p models/{my_robot,maze,obstacles}/meshes
    mkdir -p worlds/{empty,office,maze,warehouse}
    mkdir -p config/controllers

    #Create resource directories
    touch gazebo_simulation/__init__.py

🔧 Practical Exercises

Exercise 1: First Steps with Gazebo

1.1 Launch Empty World:

    #Launch Gazebo with empty world
    ign gazebo empty.sdf

    #Or use ROS 2 launch file
    ros2 launch gazebo_ros ign_gazebo.launch.py ign_args:="-r empty.sdf"
1.2 Spawn Simple Shapes:

    #!/usr/bin/env python3
    #gazebo_simulation/robots/spawn_shapes.py
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Pose
    from std_msgs.msg import String
    import random

    class ShapeSpawner(Node):
        def __init__(self):
            super().__init__('shape_spawner')
        
        # Publisher to spawn entities in Gazebo
            self.spawn_pub = self.create_publisher(String, '/spawn_entity', 10)
        
        # Timer to spawn shapes
            self.timer = self.create_timer(2.0, self.spawn_random_shape)
        
            self.shapes = ['box', 'sphere', 'cylinder', 'capsule']
            self.colors = ['red', 'green', 'blue', 'yellow', 'purple', 'cyan']
        
            self.get_logger().info("Shape Spawner started")
    
        def spawn_random_shape(self):
            shape = random.choice(self.shapes)
            color = random.choice(self.colors)
        
        # Create SDF string
            sdf = f'''
            <?xml version="1.0" ?>
            <sdf version="1.6">
                <model name="{shape}_{random.randint(1000, 9999)}">
                    <pose>
                        {random.uniform(-5, 5)} 
                        {random.uniform(-5, 5)} 
                        {random.uniform(0, 2)} 
                        0 0 {random.uniform(0, 6.28)}
                    </pose>
                    <link name="link">
                        <collision name="collision">
                            <geometry>
                                <{shape}>
                                    <size>
                                        {random.uniform(0.1, 0.5)} 
                                        {random.uniform(0.1, 0.5)} 
                                        {random.uniform(0.1, 0.5)}
                                    </size>
                                </{shape}>
                            </geometry>
                        </collision>
                        <visual name="visual">
                            <geometry>
                                <{shape}>
                                    <size>
                                        {random.uniform(0.1, 0.5)} 
                                        {random.uniform(0.1, 0.5)} 
                                        {random.uniform(0.1, 0.5)}
                                    </size>
                                </{shape}>
                            </geometry>
                            <material>
                                <ambient>{color}</ambient>
                                <diffuse>{color}</diffuse>
                                <specular>1 1 1 1</specular>
                            </material>
                        </visual>
                    </link>
                </model>
            </sdf>
            '''
        
            msg = String()
            msg.data = sdf
            self.spawn_pub.publish(msg)
            self.get_logger().info(f"Spawned {color} {shape}")

    def main(args=None):
        rclpy.init(args=args)
        node = ShapeSpawner()
    
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
  1.3 Control Gazebo from CLI:

    #List available worlds
    ign gazebo -l

    #Run headless (no GUI)
    ign gazebo -s -r empty.sdf

    #Play/Pause simulation
    ign service -s /world/empty/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req 'pause:     true'
    ign service -s /world/empty/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req 'pause:     false'

    #Set simulation speed
    ign service -s /world/empty/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req             'multi_step: 2'

    #Spawn model from SDF
    ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 3000 --req             'sdf_filename: "model.sdf"'
