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
Exercise 2: Creating Robot Models

2.1 Simple Differential Drive Robot (URDF + Xacro):

    Create urdf/simple_robot.urdf.xacro:

    <?xml version="1.0"?>
    <robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    
        <!-- Properties -->
        <xacro:property name="base_width" value="0.4"/>
        <xacro:property name="base_length" value="0.5"/>
        <xacro:property name="base_height" value="0.2"/>
        <xacro:property name="wheel_radius" value="0.1"/>
        <xacro:property name="wheel_width" value="0.05"/>
    
        <!-- Colors -->
        <material name="blue">
            <color rgba="0.2 0.5 1.0 1.0"/>
        </material>
        <material name="black">
            <color rgba="0.0 0.0 0.0 1.0"/>
        </material>
        <material name="gray">
            <color rgba="0.5 0.5 0.5 1.0"/>
        </material>
    
        <!-- Base Link -->
        <link name="base_link">
            <visual>
                <geometry>
                    <box size="${base_length} ${base_width} ${base_height}"/>
                </geometry>
                <material name="blue"/>
            </visual>
            <collision>
                <geometry>
                    <box size="${base_length} ${base_width} ${base_height}"/>
                </geometry>
            </collision>
            <inertial>
                <mass value="10.0"/>
                <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
            </inertial>
        </link>
    
        <!-- Left Wheel -->
        <link name="left_wheel_link">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
                </geometry>
                <origin rpy="0 0 0" xyz="0 0 0"/>
                <material name="black"/>
            </visual>
            <collision>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
                </geometry>
            </collision>
            <inertial>
                <mass value="1.0"/>
                <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
            </inertial>
        </link>
    
        <!-- Right Wheel -->
        <link name="right_wheel_link">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
                </geometry>
                <origin rpy="0 0 0" xyz="0 0 0"/>
                <material name="black"/>
            </visual>
            <collision>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
                </geometry>
            </collision>
            <inertial>
                <mass value="1.0"/>
                <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
            </inertial>
        </link>
    
        <!-- Caster Wheel -->
        <link name="caster_wheel_link">
            <visual>
                <geometry>
                    <sphere radius="0.05"/>
                </geometry>
                <material name="gray"/>
            </visual>
            <collision>
                <geometry>
                    <sphere radius="0.05"/>
                </geometry>
            </collision>
            <inertial>
                <mass value="0.5"/>
                <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
            </inertial>
        </link>
    
        <!-- Base Joint -->
        <joint name="base_joint" type="fixed">
            <parent link="base_link"/>
            <child link="base_link"/>
            <origin xyz="0 0 0"/>
        </joint>
    
        <!-- Left Wheel Joint -->
        <joint name="left_wheel_joint" type="continuous">
            <parent link="base_link"/>
            <child link="left_wheel_link"/>
            <origin xyz="${-base_length/3} ${-base_width/2 - wheel_width/2} 0" rpy="-1.5708 0 0"/>
            <axis xyz="0 0 1"/>
            <limit effort="10" velocity="10"/>
        </joint>
    
        <!-- Right Wheel Joint -->
        <joint name="right_wheel_joint" type="continuous">
            <parent link="base_link"/>
            <child link="right_wheel_link"/>
            <origin xyz="${-base_length/3} ${base_width/2 + wheel_width/2} 0" rpy="-1.5708 0 0"/>
            <axis xyz="0 0 1"/>
            <limit effort="10" velocity="10"/>
        </joint>
    
        <!-- Caster Wheel Joint -->
        <joint name="caster_wheel_joint" type="fixed">
            <parent link="base_link"/>
            <child link="caster_wheel_link"/>
            <origin xyz="${base_length/3} 0 -0.05"/>
        </joint>
    
        <!-- Transmission for ros2_control -->
        <transmission name="left_wheel_trans">
            <type>transmission_interface/SimpleTransmission</type>
            <joint name="left_wheel_joint">
                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
            </joint>
            <actuator name="left_wheel_motor">
                <mechanicalReduction>1</mechanicalReduction>
            </actuator>
        </transmission>
    
        <transmission name="right_wheel_trans">
            <type>transmission_interface/SimpleTransmission</type>
            <joint name="right_wheel_joint">
                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
            </joint>
            <actuator name="right_wheel_motor">
                <mechanicalReduction>1</mechanicalReduction>
            </actuator>
        </transmission>
    
        <!-- Gazebo Plugins -->
        <gazebo>
            <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
                <parameters>$(find gazebo_simulation)/config/robot_control.yaml</parameters>
            </plugin>
        
            <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
                <ros>
                    <namespace>/robot</namespace>
                    <argument>odom:=odom</argument>
                </ros>
                <update_rate>50</update_rate>
            </plugin>
        </gazebo>
    
        <!-- ros2_control Hardware Interface -->
        <ros2_control name="GazeboSystem" type="system">
            <hardware>
                <plugin>gazebo_ros2_control/GazeboSystem</plugin>
            </hardware>
            <joint name="left_wheel_joint">
                <command_interface name="velocity">
                    <param name="min">-1</param>
                    <param name="max">1</param>
                </command_interface>
                <state_interface name="position"/>
                <state_interface name="velocity"/>
            </joint>
            <joint name="right_wheel_joint">
                <command_interface name="velocity">
                    <param name="min">-1</param>
                    <param name="max">1</param>
                </command_interface>
                <state_interface name="position"/>
                <state_interface name="velocity"/>
            </joint>
        </ros2_control>
    
    </robot>
2.2 Controller Configuration:

Create config/robot_control.yaml:

    controller_manager:
      ros__parameters:
        update_rate: 50
        use_sim_time: true
    
    # Controller definitions
        joint_state_broadcaster:
          type: joint_state_broadcaster/JointStateBroadcaster
    
        diff_drive_controller:
          type: diff_drive_controller/DiffDriveController
    
        imu_sensor_broadcaster:
          type: imu_sensor_broadcaster/IMUSensorBroadcaster

    #Differential Drive Controller
    diff_drive_controller:
      ros__parameters:
        left_wheel_names: ["left_wheel_joint"]
        right_wheel_names: ["right_wheel_joint"]
    
        wheel_separation: 0.45
        wheels_per_side: 1
        wheel_radius: 0.1
    
        wheel_separation_multiplier: 1.0
        left_wheel_radius_multiplier: 1.0
        right_wheel_radius_multiplier: 1.0
    
        publish_rate: 50.0
        odom_frame_id: odom
        base_frame_id: base_link
        pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
        twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    
        open_loop: false
        enable_odom_tf: true
    
        cmd_vel_timeout: 0.5
        use_stamped_vel: true
    
        # Velocity limits
        linear.x.max_velocity: 1.0
        linear.x.min_velocity: -0.5
        linear.x.max_acceleration: 1.0
        linear.x.max_deceleration: 1.0
    
        angular.z.max_velocity: 2.0
        angular.z.min_velocity: -2.0
        angular.z.max_acceleration: 2.0
        angular.z.max_deceleration: 2.0
    
2.3 Launch File for Robot Description:

Create launch/display_robot.launch.py:

    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node
    from ament_index_python.packages import get_package_share_directory
    import os

    def generate_launch_description():
        # Get package directory
        pkg_dir = get_package_share_directory('gazebo_simulation')
    
    # URDF file path
        urdf_file = os.path.join(pkg_dir, 'urdf', 'simple_robot.urdf.xacro')
    
        # Declare launch arguments
        use_sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        )
    
        gui = DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Use joint state publisher gui'
        )
    
        # Process URDF with xacro
        from xacro import process_file
        doc = process_file(urdf_file)
        robot_description = doc.toprettyxml(indent='  ')
    
    # Robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    
        # Joint state publisher
        joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(LaunchConfiguration('gui'))
        )
    
        # Joint state publisher GUI
        joint_state_publisher_gui = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui'))
        )
    
        # RViz2
        rviz_config = os.path.join(pkg_dir, 'config', 'robot_display.rviz')
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    
        return LaunchDescription([
            use_sim_time,
            gui,
            robot_state_publisher,
            joint_state_publisher,
            joint_state_publisher_gui,
            rviz_node
        ])

2.4 Test Robot Description:

    #Display robot in RViz
    ros2 launch gazebo_simulation display_robot.launch.py

    #Check TF tree
    ros2 run tf2_tools view_frames.py

    #List joints
    ros2 topic echo /joint_states

    #Publish test joint commands
    ros2 topic pub /simple_velocity_controller/commands geometry_msgs/msg/TwistStamped "
    header:
      frame_id: base_link
    twist:
      linear:
        x: 0.5
      angular:
        z: 0.2"

Exercise 3: Spawn Robot in Gazebo

3.1 Gazebo Launch File:

Create launch/spawn_robot.launch.py:

    from launch import LaunchDescription
    from launch.actions import (
        DeclareLaunchArgument,
        IncludeLaunchDescription,
        ExecuteProcess,
        RegisterEventHandler
    )
    from launch.conditions import IfCondition
    from launch.event_handlers import OnProcessExit
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare
    import os

    def generate_launch_description():
        # Package paths
        pkg_gazebo_sim = FindPackageShare('gazebo_simulation')
        pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    
    # Launch configurations
        use_sim_time = LaunchConfiguration('use_sim_time', default='true')
        headless = LaunchConfiguration('headless', default='false')
        world = LaunchConfiguration('world', default='empty')
    
    # World file path
        world_path = PathJoinSubstitution([
            pkg_gazebo_sim, 'worlds', LaunchConfiguration('world')
        ])
    
    # URDF file path
        urdf_path = PathJoinSubstitution([
            pkg_gazebo_sim, 'urdf', 'simple_robot.urdf.xacro'
        ])
    
    # Launch arguments
        declare_use_sim_time = DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time'
        )
    
        declare_headless = DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo headless'
        )
    
        declare_world = DeclareLaunchArgument(
            'world', default_value='empty.sdf',
            description='World file name'
        )
    
    # Start Gazebo
        start_gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                pkg_gazebo_ros, '/launch/ign_gazebo.launch.py'
            ]),
            launch_arguments={
                'ign_args': ['-r -v 3 ', world_path],
                'use_sim_time': use_sim_time,
                'headless': headless
            }.items()
        )
    
    # Process URDF with xacro
        from xacro import process_file
        import tempfile
    
    # Get robot description
        robot_description_content = process_file(
            urdf_path.perform(None)
        ).toprettyxml(indent='  ')
    
    # Robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time
            }]
        )
    
    # Spawn robot in Gazebo
        spawn_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            arguments=[
                '-entity', 'simple_robot',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1',
                '-Y', '0.0'
            ],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    
        # Spawn robot after Gazebo is ready
        spawn_robot_after_gazebo = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=start_gazebo,
                on_exit=[spawn_robot]
            )
        )
    
    # Load controllers
        load_joint_state_broadcaster = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 'joint_state_broadcaster'],
            output='screen'
        )
    
        load_diff_drive_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 'diff_drive_controller'],
            output='screen'
        )
    
    # Delay controller loading after spawn
        load_controllers = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_joint_state_broadcaster, load_diff_drive_controller]
            )
        )
    
        return LaunchDescription([
            declare_use_sim_time,
            declare_headless,
            declare_world,
            start_gazebo,
            robot_state_publisher,
            spawn_robot_after_gazebo,
            load_controllers
        ])

3.2 Run Robot in Simulation:

    #Build the workspace
    cd ~/ros2_ws
    colcon build --packages-select gazebo_simulation
    source install/setup.bash

    #Launch robot in empty world
    ros2 launch gazebo_simulation spawn_robot.launch.py world:=empty.sdf

    #In another terminal, control the robot
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "
    linear:
      x: 0.5
    angular:
      z: 0.5"

    #Check robot state
    ros2 topic echo /odom
    ros2 topic echo /joint_states

    #List available controllers
    ros2 control list_controllers
    ros2 control list_hardware_interfaces

Exercise 4: Adding Sensors
4.1 Add LiDAR Sensor:

Add to urdf/simple_robot.urdf.xacro:

    <!-- LiDAR Sensor -->
    <link name="lidar_link">
        <visual>
            <geometry>
                <cylinder radius="0.05" length="0.05"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.1"/>
            <material name="gray"/>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.05" length="0.05"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.1"/>
            <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
        </inertial>
    </link>

    <joint name="lidar_joint" type="fixed">
        <parent link="base_link"/>
        <child link="lidar_link"/>
        <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
    </joint>

    <!-- Gazebo LiDAR Plugin -->
    <gazebo reference="lidar_link">
        <sensor name="lidar" type="gpu_lidar">
            <pose>0 0 0.1 0 0 0</pose>
            <topic>scan</topic>
            <update_rate>20</update_rate>
            <lidar>
                <scan>
                    <horizontal>
                        <samples>360</samples>
                        <resolution>1</resolution>
                        <min_angle>-3.14159</min_angle>
                        <max_angle>3.14159</max_angle>
                    </horizontal>
                    <vertical>
                        <samples>16</samples>
                        <resolution>1</resolution>
                        <min_angle>-0.2618</min_angle>
                        <max_angle>0.2618</max_angle>
                    </vertical>
                </scan>
                <range>
                    <min>0.1</min>
                    <max>10.0</max>
                    <resolution>0.01</resolution>
                </range>
                <noise>
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.01</stddev>
                </noise>
            </lidar>
            <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_gpu_lidar.so">
                <ros>
                    <namespace>/robot</namespace>
                    <argument>scan:=scan</argument>
                </ros>
                <output_type>sensor_msgs/LaserScan</output_type>
                <frame_name>lidar_link</frame_name>
            </plugin>
        </sensor>
    </gazebo>
4.2 Add Camera Sensor:
    
    <!-- Camera Link -->
    <link name="camera_link">
        <visual>
            <geometry>
                <box size="0.03 0.05 0.05"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <material name="black"/>
        </visual>
    </link>

    <joint name="camera_joint" type="fixed">
        <parent link="base_link"/>
        <child link="camera_link"/>
        <origin xyz="0.25 0 0.2" rpy="0 0 0"/>
    </joint>

    <!-- Camera Sensor -->
    <gazebo reference="camera_link">
        <sensor name="camera" type="camera">
            <pose>0 0 0 0 0 0</pose>
            <topic>image_raw</topic>
            <update_rate>30</update_rate>
            <camera>
                <horizontal_fov>1.0472</horizontal_fov>
                <image>
                    <width>640</width>
                    <height>480</height>
                    <format>R8G8B8</format>
                </image>
                <clip>
                    <near>0.1</near>
                    <far>100</far>
                </clip>
                <noise>
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.01</stddev>
                </noise>
            </camera>
            <plugin name="gazebo_ros_camera" filename="libgazebo_ros_camera.so">
                <ros>
                    <namespace>/robot</namespace>
                    <argument>image_raw:=image_raw</argument>
                    <argument>camera_info:=camera_info</argument>
                </ros>
                <camera_name>camera</camera_name>
                <frame_name>camera_link</frame_name>
                <distortion_k1>0.0</distortion_k1>
                <distortion_k2>0.0</distortion_k2>
                <distortion_k3>0.0</distortion_k3>
                <distortion_t1>0.0</distortion_t1>
                <distortion_t2>0.0</distortion_t2>
            </plugin>
        </sensor>
    </gazebo>
4.3 Add IMU Sensor:

    <!-- IMU Link -->
    <link name="imu_link">
        <visual>
            <geometry>
                <box size="0.02 0.02 0.02"/>
            </geometry>
            <material name="blue"/>
        </visual>
    </link>

    <joint name="imu_joint" type="fixed">
        <parent link="base_link"/>
        <child link="imu_link"/>
        <origin xyz="-0.1 0 0.1" rpy="0 0 0"/>
    </joint>

    <!-- IMU Sensor -->
    <gazebo reference="imu_link">
        <sensor name="imu" type="imu">
            <pose>0 0 0 0 0 0</pose>
            <topic>imu</topic>
            <update_rate>100</update_rate>
            <always_on>1</always_on>
            <visualize>true</visualize>
            <imu>
                <angular_velocity>
                    <x>
                        <noise type="gaussian">
                            <mean>0.0</mean>
                            <stddev>0.0002</stddev>
                            <bias_mean>0.00001</bias_mean>
                            <bias_stddev>0.00002</bias_stddev>
                        </noise>
                    </x>
                    <y>
                        <noise type="gaussian">
                            <mean>0.0</mean>
                            <stddev>0.0002</stddev>
                            <bias_mean>0.00001</bias_mean>
                            <bias_stddev>0.00002</bias_stddev>
                        </noise>
                    </y>
                    <z>
                        <noise type="gaussian">
                            <mean>0.0</mean>
                            <stddev>0.0002</stddev>
                            <bias_mean>0.00001</bias_mean>
                            <bias_stddev>0.00002</bias_stddev>
                        </noise>
                    </z>
                </angular_velocity>
                <linear_acceleration>
                    <x>
                        <noise type="gaussian">
                            <mean>0.0</mean>
                            <stddev>0.01</stddev>
                            <bias_mean>0.001</bias_mean>
                            <bias_stddev>0.001</bias_stddev>
                        </noise>
                    </x>
                    <y>
                        <noise type="gaussian">
                            <mean>0.0</mean>
                            <stddev>0.01</stddev>
                            <bias_mean>0.001</bias_mean>
                            <bias_stddev>0.001</bias_stddev>
                        </noise>
                    </y>
                    <z>
                        <noise type="gaussian">
                            <mean>0.0</mean>
                            <stddev>0.01</stddev>
                            <bias_mean>0.001</bias_mean>
                            <bias_stddev>0.001</bias_stddev>
                        </noise>
                    </z>
                </linear_acceleration>
            </imu>
            <plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu.so">
                <ros>
                    <namespace>/robot</namespace>
                    <argument>imu:=imu</argument>
                </ros>
                <frame_name>imu_link</frame_name>
                <initial_orientation_as_reference>false</initial_orientation_as_reference>
            </plugin>
        </sensor>
    </gazebo>
 4.4 Sensor Visualization Node:
Create gazebo_simulation/sensors/sensor_viewer.py:

    #!/usr/bin/env python3
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan, Image, Imu
    from nav_msgs.msg import Odometry
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    import cv2
    from cv_bridge import CvBridge
    import numpy as np
    import math

    class SensorViewer(Node):
        def __init__(self):
            super().__init__('sensor_viewer')
        
        # Subscribers
            self.scan_sub = self.create_subscription(
                LaserScan, '/robot/scan', self.scan_callback, 10)
        
            self.image_sub = self.create_subscription(
                Image, '/robot/image_raw', self.image_callback, 10)
        
            self.imu_sub = self.create_subscription(
                Imu, '/robot/imu', self.imu_callback, 10)
        
            self.odom_sub = self.create_subscription(
                Odometry, '/robot/odom', self.odom_callback, 10)
        
        # Publishers
            self.scan_marker_pub = self.create_publisher(
                MarkerArray, '/robot/scan_markers', 10)
        
        # Bridge for image conversion
            self.bridge = CvBridge()
        
        # Sensor data storage
            self.latest_scan = None
            self.latest_image = None
            self.latest_imu = None
            self.latest_odom = None
        
        # Timer for visualization
            self.create_timer(0.1, self.publish_visualization)
        
            self.get_logger().info("Sensor Viewer started")
    
        def scan_callback(self, msg):
            self.latest_scan = msg
    
        def image_callback(self, msg):
            self.latest_image = msg
        
        # Display image in OpenCV window
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                cv2.imshow('Robot Camera', cv_image)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().error(f'Image conversion error: {e}')
    
        def imu_callback(self, msg):
            self.latest_imu = msg
        
        # Log IMU data
            self.get_logger().info(
                f'IMU - Orientation: {msg.orientation.w:.2f}, '
                f'Angular Vel: {msg.angular_velocity.z:.2f}',
                throttle_duration_sec=1.0
            )
    
        def odom_callback(self, msg):
            self.latest_odom = msg
    
        def publish_visualization(self):
            if self.latest_scan:
                self.publish_scan_markers()
    
        def publish_scan_markers(self):
            marker_array = MarkerArray()
        
            scan = self.latest_scan
            angle = scan.angle_min
        
            for i, range_val in enumerate(scan.ranges):
                if range_val < scan.range_min or range_val > scan.range_max:
                    angle += scan.angle_increment
                    continue
            
                # Calculate point position
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
            
            # Create marker
                marker = Marker()
                marker.header.frame_id = 'lidar_link'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'scan_points'
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
            
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.0
            
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
            
            # Color based on distance
                intensity = min(1.0, range_val / 5.0)
                marker.color.r = intensity
                marker.color.g = 1.0 - intensity
                marker.color.b = 0.0
                marker.color.a = 0.8
            
                marker.lifetime.sec = 1
            
                marker_array.markers.append(marker)
                angle += scan.angle_increment
        
            self.scan_marker_pub.publish(marker_array)

    def main(args=None):
        rclpy.init(args=args)
        node = SensorViewer()
    
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
4.5 Test Sensors:

    #Launch robot with sensors
    ros2 launch gazebo_simulation spawn_robot.launch.py world:=empty.sdf

    #In another terminal, start sensor viewer
    ros2 run gazebo_simulation sensor_viewer

    #Check sensor topics
    ros2 topic list | grep -E "/robot/(scan|image_raw|imu|odom)"

    #View LiDAR data
    ros2 topic echo /robot/scan --once

    #View camera in RViz
    rviz2 -d $(ros2 pkg prefix gazebo_simulation)/share/gazebo_simulation/config/sensor_display.rviz

    
Exercise 5: Creating Custom Worlds
5.1 Simple Maze World:

Create worlds/maze.sdf:

    <?xml version="1.0" ?>
    <sdf version="1.8">
        <world name="maze">
            <!-- Physics -->
            <physics name="default_physics" type="ode">
                <gravity>0 0 -9.8</gravity>
                <max_step_size>0.001</max_step_size>
                <real_time_factor>1.0</real_time_factor>
                <real_time_update_rate>1000</real_time_update_rate>
            </physics>
        
            <!-- Scene settings -->
            <scene>
                <ambient>0.4 0.4 0.4 1</ambient>
                <background>0.7 0.7 0.7 1</background>
                <shadows>true</shadows>
            </scene>
        
            <!-- Light -->
            <light name="sun" type="directional">
                <pose>0 0 10 0 0 0</pose>
                <diffuse>0.8 0.8 0.8 1</diffuse>
                <specular>0.2 0.2 0.2 1</specular>
                <direction>-0.5 0.1 -0.9</direction>
                <attenuation>
                    <range>100</range>
                    <constant>0.9</constant>
                    <linear>0.01</linear>
                    <quadratic>0.001</quadratic>
                </attenuation>
                <cast_shadows>true</cast_shadows>
            </light>
        
            <!-- Ground plane -->
            <include>
                <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Ground Plane</uri>
                <pose>0 0 0 0 0 0</pose>
            </include>
        
            <!-- Maze walls -->
            <!-- Outer walls -->
            <model name="wall_north">
                <pose>0 5 0.5 0 0 0</pose>
                <static>true</static>
                <link name="link">
                    <collision name="collision">
                        <geometry>
                            <box>
                                <size>12 0.2 1</size>
                            </box>
                        </geometry>
                    </collision>
                    <visual name="visual">
                        <geometry>
                            <box>
                                <size>12 0.2 1</size>
                            </box>
                        </geometry>
                        <material>
                            <ambient>0.5 0.3 0.1 1</ambient>
                            <diffuse>0.8 0.6 0.2 1</diffuse>
                        </material>
                    </visual>
                </link>
            </model>
        
            <model name="wall_south">
                <pose>0 -5 0.5 0 0 0</pose>
                <static>true</static>
                <link name="link">
                    <collision name="collision">
                        <geometry>
                            <box>
                                <size>12 0.2 1</size>
                            </box>
                        </geometry>
                    </collision>
                    <visual name="visual">
                        <geometry>
                            <box>
                                <size>12 0.2 1</size>
                            </box>
                        </geometry>
                        <material>
                            <ambient>0.5 0.3 0.1 1</ambient>
                            <diffuse>0.8 0.6 0.2 1</diffuse>
                        </material>
                    </visual>
                </link>
            </model>
        
            <model name="wall_east">
                <pose>6 0 0.5 0 0 1.5708</pose>
                <static>true</static>
                <link name="link">
                    <collision name="collision">
                        <geometry>
                            <box>
                                <size>10 0.2 1</size>
                            </box>
                        </geometry>
                    </collision>
                    <visual name="visual">
                        <geometry>
                            <box>
                                <size>10 0.2 1</size>
                            </box>
                        </geometry>
                        <material>
                            <ambient>0.5 0.3 0.1 1</ambient>
                            <diffuse>0.8 0.6 0.2 1</diffuse>
                        </material>
                    </visual>
                </link>
            </model>
        
            <model name="wall_west">
                <pose>-6 0 0.5 0 0 1.5708</pose>
                <static>true</static>
                <link name="link">
                    <collision name="collision">
                        <geometry>
                            <box>
                                <size>10 0.2 1</size>
                            </box>
                        </geometry>
                    </collision>
                    <visual name="visual">
                        <geometry>
                            <box>
                                <size>10 0.2 1</size>
                            </box>
                        </geometry>
                        <material>
                            <ambient>0.5 0.3 0.1 1</ambient>
                            <diffuse>0.8 0.6 0.2 1</diffuse>
                        </material>
                    </visual>
                </link>
            </model>
        
            <!-- Internal maze walls -->
            <model name="wall_1">
                <pose>-2 -2 0.5 0 0 0</pose>
                <static>true</static>
                <link name="link">
                    <collision name="collision">
                        <geometry>
                            <box>
                                <size>4 0.2 1</size>
                            </box>
                        </geometry>
                    </collision>
                    <visual name="visual">
                        <geometry>
                            <box>
                                <size>4 0.2 1</size>
                            </box>
                        </geometry>
                        <material>
                            <ambient>0.5 0.3 0.1 1</ambient>
                            <diffuse>0.8 0.6 0.2 1</diffuse>
                        </material>
                    </visual>
                </link>
            </model>
        
            <model name="wall_2">
                <pose>2 2 0.5 0 0 1.5708</pose>
                <static>true</static>
                <link name="link">
                    <collision name="collision">
                        <geometry>
                            <box>
                                <size>4 0.2 1</size>
                            </box>
                        </geometry>
                    </collision>
                    <visual name="visual">
                        <geometry>
                            <box>
                                <size>4 0.2 1</size>
                            </box>
                        </geometry>
                        <material>
                            <ambient>0.5 0.3 0.1 1</ambient>
                            <diffuse>0.8 0.6 0.2 1</diffuse>
                        </material>
                    </visual>
                </link>
            </model>
        
            <!-- Obstacles -->
            <model name="obstacle_1">
                <pose>1 -1 0.3 0 0 0</pose>
                <static>true</static>
                <link name="link">
                    <collision name="collision">
                        <geometry>
                            <box>
                                <size>0.5 0.5 0.6</size>
                            </box>    
                        </geometry>
                    </collision>
                    <visual name="visual">
                        <geometry>
                            <box>
                                <size>0.5 0.5 0.6</size>
                            </box>
                        </geometry>
                        <material>
                            <ambient>1 0 0 1</ambient>
                            <diffuse>0.8 0 0 1</diffuse>
                        </material>
                    </visual>
                </link>
            </model>
        
            <model name="obstacle_2">
                <pose>-1 3 0.3 0 0 0</pose>
                <static>true</static>
                <link name="link">
                    <collision name="collision">
                        <geometry>
                            <cylinder>
                                <radius>0.3</radius>
                                <length>0.6</length>
                            </cylinder>
                        </geometry>
                    </collision>
                    <visual name="visual">
                        <geometry>
                            <cylinder>
                                <radius>0.3</radius>
                                <length>0.6</length>
                            </cylinder>
                        </geometry>
                        <material>
                            <ambient>0 1 0 1</ambient>
                            <diffuse>0 0.8 0 1</diffuse>
                        </material>
                    </visual>
                </link>
            </model>
        
            <!-- Goal area -->
            <model name="goal">
                <pose>4 -3 0.01 0 0 0</pose>
                <static>true</static>
                <link name="link">
                    <collision name="collision">
                        <geometry>
                            <box>
                                <size>1 1 0.02</size>
                            </box>
                        </geometry>
                    </collision>
                    <visual name="visual">
                        <geometry>
                            <box>
                                <size>1 1 0.02</size>
                            </box>
                        </geometry>
                        <material>
                            <ambient>1 1 0 1</ambient>
                            <diffuse>1 1 0 0.8</diffuse>
                            <emissive>0.5 0.5 0 1</emissive>
                        </material>
                    </visual>
                </link>
            </model>
        
            <!-- GUI settings -->
            <gui fullscreen="0">
                <camera name="user_camera">
                    <pose>-8 0 5 0 0.6 1.57</pose>
                    <view_controller>orbit</view_controller>
                </camera>
            </gui>
        </world>        
    </sdf>
5.2 Office World with Furniture:

Create worlds/office.sdf:

    <?xml version="1.0" ?>
    <sdf version="1.8">
        <world name="office">
            <physics>
                <gravity>0 0 -9.8</gravity>
                <max_step_size>0.001</max_step_size>
                <real_time_factor>1</real_time_factor>
            </physics>
        
            <scene>
                <ambient>0.4 0.4 0.4 1</ambient>
                <background>0.8 0.8 0.8 1</background>
                <shadows>true</shadows>
                <grid>true</grid>
            </scene>
        
            <light name="ceiling_light" type="directional">
                <pose>0 0 5 0 0 0</pose>
                <diffuse>1 1 1 1</diffuse>
                <specular>0.5 0.5 0.5 1</specular>
                <direction>-0.2 -0.2 -1</direction>
                <cast_shadows>true</cast_shadows>
            </light>
        
            <include>
                <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Ground Plane</uri>
                <pose>0 0 0 0 0 0</pose>
            </include>
        
            <!-- Office Furniture -->
            <include>
                <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Desk</uri>
                <pose>2 1 0 0 0 1.57</pose>
            </include>
        
            <include>
                <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Chair</uri>
                <pose>2 0.5 0 0 0 0</pose>
            </include>
        
            <include>
                <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Bookshelf</uri>
                <pose>-2 -1 0 0 0 0</pose>
            </include>
        
            <include>
                <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Cabinet</uri>
                <pose>0 -2 0 0 0 0</pose>
            </include>
        
            <include>
                <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Table</uri>
                <pose>-2 2 0 0 0 1.57</pose>
            </include>
        
            <!-- People -->
            <include>
                <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Person Standing</uri>
                <pose>3 -1 0 0 0 0</pose>
            </include>
        
            <include>
                <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Person Sitting</uri>
                <pose>-1 3 0 0 0 0</pose>
            </include>
        
            <!-- Plants -->
            <include>
                <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Plant</uri>
                <pose>4 2 0 0 0 0</pose>
            </include>
        
            <include>
                <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Plant</uri>
                <pose>-4 -2 0 0 0 0</pose>
            </include>
        
            <!-- Trash bin -->
            <include>
                <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Trash Bin</uri>
                <pose>3.5 -3 0 0 0 0</pose>
            </include>
        
            <!-- Computer monitors on desks -->
            <model name="monitor_1">
                <pose>2 1.2 0.8 0 0 1.57</pose>
                <static>true</static>
                <link name="link">
                    <visual name="screen">
                        <geometry>
                            <box>
                                <size>0.5 0.05 0.4</size>
                            </box>
                        </geometry>
                        <material>
                            <ambient>0.1 0.1 0.1 1</ambient>
                            <diffuse>0.2 0.2 0.2 1</diffuse>
                            <emissive>0.1 0.1 0.1 1</emissive>
                        </material>
                    </visual>
                    <visual name="stand">
                        <pose>0 -0.15 -0.2 0 0 0</pose>
                        <geometry>
                            <cylinder>
                                <radius>0.05</radius>
                                <length>0.2</length>
                            </cylinder>
                        </geometry>
                        <material>
                            <ambient>0.3 0.3 0.3 1</ambient>
                        </material>
                    </visual>
                </link>
            </model>
        
            <!-- Fire alarm -->
            <model name="fire_alarm">
                <pose>-2 0 0.5 0 0 0</pose>
                <static>true</static>
                <link name="link">
                    <visual name="base">
                        <geometry>
                            <box>
                                <size>0.1 0.1 0.05</size>
                            </box>
                        </geometry>
                        <material>
                            <ambient>1 0 0 1</ambient>
                            <emissive>1 0 0 0.3</emissive>
                        </material>
                    </visual>
                </link>
            </model>
        </world>
    </sdf>
