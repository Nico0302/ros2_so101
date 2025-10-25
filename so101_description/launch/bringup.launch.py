# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

default_controllers = [
    "joint_state_broadcaster",
    "gripper_controller",
]

PACKAGE_NAME = "so101_description"

def generate_launch_description():

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value=PACKAGE_NAME,
            description='Package containing the URDF in the "urdf" folder. Usually the argument is not set, \
                         it enables use of a custom description.',
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="so101.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "controllers_package",
            default_value=PACKAGE_NAME,
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="controller_manager.yaml",
            description="YAML file with the controllers configuration.",
        ),
        DeclareLaunchArgument(
            "arm_controller",
            default_value="so101_arm_controller",
            description="Robot arm controller to start.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start Rviz2 automatically.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
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
    ]

    # Configuration variables
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    arm_controller = LaunchConfiguration("arm_controller")
    controllers_package = LaunchConfiguration("controllers_package")
    controllers_file = LaunchConfiguration("controllers_file")
    use_rviz = LaunchConfiguration("use_rviz")
    prefix = LaunchConfiguration("prefix")
    use_sim = LaunchConfiguration("use_sim")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    usb_port = LaunchConfiguration("usb_port")

    # URDF generation
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare(description_package), "urdf", description_file]
                ),
                " ",
                "prefix:=",
                prefix,
                " ",
                "use_sim:=",
                use_sim,
                " ",
                "use_fake_hardware:=",
                use_fake_hardware,
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
            FindPackageShare(controllers_package),
            "config",
            controllers_file,
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(PACKAGE_NAME), "rviz", "so101.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    nodes = [
        robot_state_publisher_node,
        rviz_node,
        control_node,
        static_tf_node,
    ] 

    for controller in default_controllers + [arm_controller]:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
            )
        )

    return LaunchDescription(declared_arguments + nodes)