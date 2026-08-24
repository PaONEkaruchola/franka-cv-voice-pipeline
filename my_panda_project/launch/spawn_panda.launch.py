import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    my_pkg_dir = get_package_share_directory('my_panda_project')
    
    world_file = os.path.join(my_pkg_dir, 'worlds', 'workspace.sdf')
    
    # 1. Start Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 2. Process the custom Xacro file
    xacro_file = os.path.join(my_pkg_dir, 'urdf', 'panda_sim.urdf.xacro')
    
    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    }

    # 3. Start Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 4. Spawn the robot in Gazebo (Forcing the Home Pose)
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'panda',
                   '-allow_renaming', 'true',
                   '-z', '0.0',
                   '-J', 'panda_joint1', '0.0',
                   '-J', 'panda_joint2', '-0.785',
                   '-J', 'panda_joint3', '0.0',
                   '-J', 'panda_joint4', '-2.356',
                   '-J', 'panda_joint5', '0.0',
                   '-J', 'panda_joint6', '1.57',
                   '-J', 'panda_joint7', '0.785']
    )

    # 5. Spawn the Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # 6. Spawn the Arm Trajectory Controller
    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller"],
    )
    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller"],
    )


    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        panda_hand_controller_spawner
    ])