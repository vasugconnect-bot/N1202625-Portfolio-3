from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    maze_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ntu_robotsim_octomap"),
                "launch",
                "maze.launch.py"
            ])
        )
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ntu_robotsim_octomap"),
                "launch",
                "single_robot_sim.launch.py"
            ])
        )
    )

    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("odom_to_tf_ros2"),
                "launch",
                "odom_to_tf.launch.py"
            ])
        )
    )

    octomap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ntu_robotsim_octomap"),
                "launch",
                "octomap_filtered.launch.py"
            ])
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py"
            ])
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": "/home/ntu-user/ros2_ws/src/ntu_robotsim_octomap/ntu_robotsim_octomap/config/nav2/nav2_octomap_params.yaml"
        }.items()
    )

    return LaunchDescription([
        maze_launch,
        robot_launch,
        tf_launch,
        octomap_launch,
        nav2_launch
    ])
