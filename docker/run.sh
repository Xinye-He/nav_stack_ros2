#!/usr/bin/env bash
#
# Start an instance of the jetson-inference docker container.
# See below or run this script with -h or --help to see usage options.
#
# This script should be run from the root dir of the jetson-inference project:
#
#     $ cd /path/to/your/jetson-inference
#     $ docker/run.sh
#

die() {
    printf '%s\n' "$1"
    show_help
    exit 1
}

# paths to some project directories
NETWORKS_DIR="data/networks"
DOCKER_ROOT="/root"	# where the project resides inside docker

# parse user arguments
USER_COMMAND=""
USER_VOLUME=""
DEV_VOLUME=""
ROS_DISTRO=""
CONTAINER_IMAGE="xinye30/ucar-jetson:nav-stack-ros2"

# check for V4L2 devices
V4L2_DEVICES=""

for i in {0..9}
do
	if [ -a "/dev/video$i" ]; then
		V4L2_DEVICES="$V4L2_DEVICES --device /dev/video$i "
	fi
done

# check for display
DISPLAY_DEVICE=""

if [ -n "$DISPLAY" ]; then
	xhost +si:localuser:root
	DISPLAY_DEVICE=" -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix "
fi

# print configuration
print_var() 
{
	if [ -n "${!1}" ]; then                                                # reference var by name - https://stackoverflow.com/a/47768983
		local trimmed="$(echo -e "${!1}" | sed -e 's/^[[:space:]]*//')"   # remove leading whitespace - https://stackoverflow.com/a/3232433    
		printf '%-17s %s\n' "$1:" "$trimmed"                              # justify prefix - https://unix.stackexchange.com/a/354094
	fi
}

print_var "CONTAINER_IMAGE"
print_var "ROS_DISTRO"
print_var "DATA_VOLUME"
print_var "DEV_VOLUME"
print_var "USER_VOLUME"
print_var "USER_COMMAND"
print_var "V4L2_DEVICES"
print_var "DISPLAY_DEVICE"


cat /proc/device-tree/model > /tmp/nv_jetson_model

docker run --runtime nvidia -it --rm \
	--privileged \
	--name nav_stack_ros2 \
	--network host \
	-v /tmp/argus_socket:/tmp/argus_socket \
	-v /etc/enctune.conf:/etc/enctune.conf \
	-v /etc/nv_tegra_release:/etc/nv_tegra_release \
	-v /tmp/nv_jetson_model:/tmp/nv_jetson_model \
	-v /var/run/dbus:/var/run/dbus \
	-v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket \
	-v /home/xinye30/lidar_rosbag:/root/lidar_rosbag \
	-v /home/xinye30/nav_stack_ros2/stack_can_ws:/root/stack_can_ws \
	--volume /dev:/dev \
	--device /dev \
	--group-add dialout \
	-w $DOCKER_ROOT \
	$DISPLAY_DEVICE $V4L2_DEVICES \
	$DATA_VOLUME $USER_VOLUME $DEV_VOLUME \
	$CONTAINER_IMAGE $USER_COMMAND
