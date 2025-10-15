from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    usb_port_arg = DeclareLaunchArgument("usb_port", default_value="")

    vive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [get_package_share_directory("cartesian_teleoperation"), "launch", "vive.launch.py"]
            )
        )
    )

    so101_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [get_package_share_directory("so101_description"), "launch", "bringup.launch.py"]
            )
        ),
        launch_arguments={"usb_port": LaunchConfiguration("usb_port")}.items(),
    )

    teleop_node = Node(
        package="cartesian_teleoperation",
        executable="teleoperation",
        output="screen",
    )

    return LaunchDescription([teleop_node])
