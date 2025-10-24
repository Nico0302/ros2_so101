#!/bin/bash

echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/src/ros2_so101/scripts/ros_aliases.sh" >> ~/.bashrc

# Start xrdp
# sudo /usr/sbin/xrdp-sesman
# sudo /usr/sbin/xrdp
