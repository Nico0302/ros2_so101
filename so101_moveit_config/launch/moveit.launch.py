from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Rviz2 automatically.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup.",
        ),
        DeclareLaunchArgument(
            "usb_port",
            default_value="/dev/ttyACM0",
            description="USB port for the robot.",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Use Gazebo sim",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use mock system",
        ),
        DeclareLaunchArgument(
            "use_servo",
            default_value="true",
            description="Setup moveit servo node",
        ),
    ]

    # Configuration variables
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")
    usb_port = LaunchConfiguration("usb_port")
    use_sim = LaunchConfiguration("use_sim")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_servo = LaunchConfiguration("use_servo")

    so101_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("so101_description"), "launch", "bringup.launch.py"]
            )
        ),
        launch_arguments={"prefix": prefix, "usb_port": usb_port, "use_sim": use_sim, "use_fake_hardware": use_fake_hardware, "start_rviz": "false"}.items(),
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "so101", package_name="so101_moveit_config"
        )
        .to_moveit_configs()
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("so101_moveit_config"), "rviz", "moveit.rviz"]
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(gui),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("so101_moveit_config")
        .yaml("config/moveit_servo.yaml")
        .to_dict()
    }
    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "so101_arm"}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
        condition=IfCondition(use_servo),
    )

    return LaunchDescription(declared_arguments + [so101_bringup_launch, run_move_group_node, rviz_node, servo_node])

