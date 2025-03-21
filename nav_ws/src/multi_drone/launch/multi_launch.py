from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multi_drone',
            executable='multi_takeoff_1',
            name='offboard_control_takeoff_and_land1',
            output='screen'
        ),
        Node(
            package='multi_drone',
            executable='multi_takeoff_2',
            name='offboard_control_takeoff_and_land2',
            output='screen'
        ),
        Node(
            package='multi_drone',
            executable='multi_takeoff_3',
            name='offboard_control_takeoff_and_land3',
            output='screen'
        ),
        Node(
            package='multi_drone',
            executable='multi_takeoff_4',
            name='offboard_control_takeoff_and_land4',
            output='screen'
        ),
        Node(
            package='multi_drone',
            executable='multi_takeoff_5',
            name='offboard_control_takeoff_and_land5',
            output='screen'
        ),
        Node(
            package='multi_drone',
            executable='multi_takeoff_6',
            name='offboard_control_takeoff_and_land6',
            output='screen'
        ),
        Node(
            package='multi_drone',
            executable='multi_takeoff_7',
            name='offboard_control_takeoff_and_land7',
            output='screen'
        ),
        Node(
            package='multi_drone',
            executable='multi_takeoff_8',
            name='offboard_control_takeoff_and_land8',
            output='screen'
        ),

    ])
