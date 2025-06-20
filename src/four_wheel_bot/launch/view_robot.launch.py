from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare("four_wheel_bot")
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "four_wheel_bot.xacro"])
    world_file = PathJoinSubstitution([pkg_share, "worlds", "wall_world.world"])

    return LaunchDescription([
        # Start Gazebo with custom world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"
                ])
            ]),
            launch_arguments={"world": world_file}.items()
        ),

        # Start robot_state_publisher with robot_description and sim time
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "robot_description": Command([
                    FindExecutable(name="xacro"), " ", xacro_file
                ])
            }]
        ),

        # Spawn robot in Gazebo
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "four_wheel_bot",
                "-topic", "robot_description",
                "-x", "0", "-y", "0", "-z", "0.1"
            ],
            output="screen"
        ),

        # Custom obstacle stop node
        Node(
            package="four_wheel_bot",
            executable="stopper",
            name="obstacle_stopper",
            output="screen"
        )
    ])

