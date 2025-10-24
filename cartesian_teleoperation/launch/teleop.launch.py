from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    package_path = FindPackageShare('cartesian_teleoperation')

    arguments = [
        DeclareLaunchArgument("usb_port", default_value="")
    ]

    usb_port = LaunchConfiguration("usb_port")

    vive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_path, "launch", "vive.launch.py"]
            )
        )
    )

    so101_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_description"), "launch", "bringup.launch.py"]
            )
        ),
        launch_arguments={"usb_port": usb_port, "controllers": "false"}.items(),
    )

    so101_description_path = FindPackageShare('so101_description')

    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [so101_description_path, "urdf", "so101.urdf.xacro"]
                ),
                " ",
                "usb_port:=",
                usb_port
            ]
        ),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            package_path,
            "config",
            "controller_manager.yaml",
        ]
    )

    teleop_node = Node(
        package="cartesian_teleoperation",
        executable="pose",
        output="screen",
        parameters=[{"target_pose_topic": "target_frame"}]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("motion_control_handle/target_frame", "target_frame"),
            ("cartesian_motion_controller/target_frame", "target_frame"),
            ("cartesian_compliance_controller/target_frame", "target_frame"),
            ("cartesian_force_controller/target_wrench", "target_wrench"),
            ("cartesian_compliance_controller/target_wrench", "target_wrench"),
            ("cartesian_force_controller/ft_sensor_wrench", "ft_sensor_wrench"),
            ("cartesian_compliance_controller/ft_sensor_wrench", "ft_sensor_wrench"),
        ],
    )

    nodes = [
        so101_bringup_launch,
        teleop_node,
        control_node
    ]

    for controller in ["joint_state_broadcaster", "cartesian_motion_controller", "gripper_controller"]:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
            )
        )

    return LaunchDescription(arguments + nodes)
