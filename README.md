# ROS 2 SO-101

ROS 2 descriptions for the SO-101 arm.

## Setup

### DevContainer

Simply open the repository in VS Code with the [Dev Container plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

### Manual

Make sure you have ROS 2 Jazzy installed and run the following commands in your workspace after cloning the repository:

```
rosdep update && rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
```

## Usage

1. Make sure to calibrate your SO-101 follower with the [LeRobot calibration](https://huggingface.co/docs/lerobot/en/so101#calibrate) script:

```bash
lerobot-calibrate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=so101_follower
```

2. Launch the control nodes and RViz:

```bash
ros2 launch so101_description bringup.launch.py usb_port:=/dev/ttyACM1
```

Alternatively, you can also launch with a fake hardware system using the `use_fake_hardware:=true` flag.

To launch the MoveIt setup run the following launch file:

```bash
ros2 launch so101_moveit_config moveit.launch.py use_fake_hardware:=true
```


## Acknowledgements

- [@TheRobotStudio](https://github.com/TheRobotStudio) for their open source SO-100 & SO-101 arms
- [@JafarAbdi](https://github.com/JafarAbdi) for his [`feetech_ros2_driver`](https://github.com/JafarAbdi/feetech_ros2_driver) an work on the ROS 2 SO-100 description (this repository reimplements the robot description and moveit config from scratch based on the new SO-101 URDF)
- [@ROBOTIS](https://github.com/ROBOTIS-GIT) for using their [Open Manipulator ROS 2 Package](https://github.com/ROBOTIS-GIT/open_manipulator) as reference 
