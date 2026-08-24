from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Automatically load the Panda's URDF, SRDF, and kinematics configuration
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_moveit_configs()

    # 1. The Brain: Launch the official MoveIt 2 motion planning backend
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
    )

    # 2. The Muscle: Launch your custom C++ Action Client
    kinematics_node = Node(
        package="my_panda_project",
        executable="panda_kinematics",
        output="screen",
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
    )

    return LaunchDescription([move_group_node, kinematics_node])