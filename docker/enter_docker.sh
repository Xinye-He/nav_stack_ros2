#!/bin/bash

# 功能: 进入 nav_stack_ros2 容器，cd 到工作空间，source 环境，并保持交互 shell

docker exec -it nav_stack_ros2 bash -c "
  cd /root/stack_can_ws && \
  echo '$(pwd)' && \
  source /opt/ros/humble/install/setup.bash && \
  source install/setup.bash && \
  source install/stack_msgs/share/stack_msgs/local_setup.bash && \
  exec bash
"
