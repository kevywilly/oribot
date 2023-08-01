#!/bin/bash

# check for V4L2 devices
for i in {0..9}
do
V4L2_DEVICES=""
    if [ -a "/dev/video$i" ]; then
		V4L2_DEVICES="$V4L2_DEVICES --device /dev/video$i "
	fi
done

for i in {0..9}
do
    I2C_DEVICES=""
    if [ -a "/dev/i2c-$i" ]; then
		I2C_DEVICES="$I2C_DEVICES --device /dev/i2c-$i "
	fi
done

# check for display
DISPLAY_DEVICE=""

if [ -n "$DISPLAY" ]; then
	#sudo xhost +si:localuser:root
	DISPLAY_DEVICE=" -e DISPLAY=localhost:10.0 -v /tmp/.X11-unix/:/tmp/.X11-unix -v /tmp/.docker.xauth:/tmp/.docker.xauth -e XAUTHORITY=/tmp/.docker.xauth " 
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

	sudo docker run --runtime nvidia -it --rm \
		--network host \
        --name orin \
		-v /tmp/argus_socket:/tmp/argus_socket \
		-v /etc/enctune.conf:/etc/enctune.conf \
		-v /etc/nv_tegra_release:/etc/nv_tegra_release \
		-v /tmp/nv_jetson_model:/tmp/nv_jetson_model \
		$DISPLAY_DEVICE $V4L2_DEVICES $I2C_DEVICES \
		$DATA_VOLUME $USER_VOLUME $DEV_VOLUME \
		kevywilly/orin-ros2-iron:latest /bin/bash
