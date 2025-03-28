from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare parameters
        DeclareLaunchArgument(
            'force_frame',
            default_value='sensor_body',
            description="Frame in which the forces are recorded. "
        ),
        DeclareLaunchArgument(
            "target_frame",
            default_value="tibia_body",
            description="Frame in which the forces need to be displayed. "
        ),
        DeclareLaunchArgument(
            "debug_mode",
            default_value='false',
            description="If enabled displays debug information. "
        ),

        # Launch Node
        Node(
            package="force_visualization",
            executable="force_visualization",
            name="force_visualization",
            output="screen", 
            parameters=[{
                "force_frame": LaunchConfiguration("force_frame"),
                "target_frame": LaunchConfiguration("target_frame"),
                "debug_mode": LaunchConfiguration("debug_mode")
            }]
        )
    ])