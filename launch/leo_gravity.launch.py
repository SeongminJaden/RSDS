from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 
                 '-s', 'libgazebo_ros_factory.so',
                 '$(find-pkg-share gazebo_leo_gravity)/worlds/leo_test.world'],
            output='screen'
        )
    ])
