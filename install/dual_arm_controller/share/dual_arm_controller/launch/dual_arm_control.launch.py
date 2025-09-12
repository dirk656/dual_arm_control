import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directory
    dummy2_moveit_config_dir = get_package_share_directory('dummy2_arm_moveit_config')

    # Launch MoveIt demo
    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dummy2_moveit_config_dir, 'launch', 'demo.launch.py')
        ),
        launch_arguments={
            'use_rviz': 'true',
            'use_sim_time': 'false'
        }.items()
    )

    # Dual arm controller node
    dual_arm_controller_node = Node(
        package='dual_arm_controller',
        executable='dual_arm_controller_node',
        name='dual_arm_controller',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        moveit_demo_launch,
        dual_arm_controller_node
    ])
