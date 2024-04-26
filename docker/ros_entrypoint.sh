#!/bin/bash
 
set -e

# Ros build
source "/opt/ros/melodic/setup.bash"
source "/root/livox_ws/devel/setup.bash"


# Libray install if you want

echo "================Gril-Calib Docker Env Ready================"

cd /root/catkin_ws

exec "$@"
