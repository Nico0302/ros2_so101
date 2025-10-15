#!/bin/bash

# Some aliases to quickly get compile source and navigate to project
alias proj="cd ~/ros2_ws/src"
alias cb="cd ~/ros2_ws \
    && colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install  \
    && proj"
alias sw="source /opt/ros/$ROS_DISTRO/setup.bash && source ~/ros2_ws/install/setup.bash"