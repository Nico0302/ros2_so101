from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    usb_port = LaunchConfiguration("usb_port")
    use_sim_time = LaunchConfiguration("use_sim_time")

    so101_description_path = FindPackageShare('so101_description')

    xacro_mappings = {
        "usb_port": "xxx.yyy.zzz.www",
        "use_fake_hardware": "true",
        "gripper": "robotiq_2f_85",
        "dof": "7",
    }

    robot_controllers = PathJoinSubstitution(
        [
            so101_description_path,
            "config",
            "controller_manager.yaml",
        ]
    )

    moveit_config = (
    MoveItConfigsBuilder(
        "so101", package_name="so101_moveit_config"
    )
    .robot_description(mappings=xacro_mappings)
    .trajectory_execution(file_path=robot_controllers)
    .planning_scene_monitor(
        publish_robot_description=True, publish_robot_description_semantic=True
    )
    .planning_pipelines(
        pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
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
        [FindPackageShare("so101_moveit_config"), "launch", "moveit.rviz"]
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    nodes = [
        robot_state_publisher_node,
        rviz_node,
        control_node,
        static_tf,
    ] 

    for controller in default_controllers:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
            )
        )

    
    
