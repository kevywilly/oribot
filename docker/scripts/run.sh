#!/bin/bash

CONTAINER_IMAGE=kevywilly/orin-ros2-iron:latest
CONTAINER_NAME=orin_ros

while :; do
    case $1 in
        -h|-\?|--help)
            show_help
            exit
            ;;
        -c|--container)  # takes an option argument; ensure it has been specified.
            if [ "$2" ]; then
                CONTAINER_IMAGE=$2
                shift
            else
                die 'ERROR: "--container" requires a non-empty option argument.'
            fi
            ;;
        --container=?*)
            CONTAINER_IMAGE=${1#*=} # delete everything up to "=" and assign the remainder.
            ;;
        --container=)  # handle the case of an empty flag
            die 'ERROR: "--container" requires a non-empty option argument.'
            ;;
	-n|--name)  # takes an option argument; ensure it has been specified.
            if [ "$2" ]; then
                CONTAINER_NAME=$2
                shift
            else
                die 'ERROR: "--container" requires a non-empty option argument.'
            fi
            ;;
        --name=?*)
            CONTAINER_NAME=${1#*=} # delete everything up to "=" and assign the remainder.
            ;;
        --name=)  # handle the case of an empty flag
            die 'ERROR: "--name" requires a non-empty option argument.'
            ;;
        --dev)
            DEV_VOLUME=" --volume $PWD:$DOCKER_ROOT "
            ;;
        -v|--volume)
            if [ "$2" ]; then
                USER_VOLUME="$USER_VOLUME --volume $2 "
                shift
            else
                die 'ERROR: "--volume" requires a non-empty option argument.'
            fi
            ;;
        --volume=?*)
            USER_VOLUME="$USER_VOLUME --volume ${1#*=} "
            ;;
        --volume=)
            die 'ERROR: "--volume" requires a non-empty option argument.'
            ;;
        --ros)
            if [ "$2" ]; then
                ROS_DISTRO=$2
                shift
            else
                ROS_DISTRO="foxy"
            fi
            ;;
        --ros=?*)
            ROS_DISTRO=${1#*=}
            ;;
        --ros=)
            die 'ERROR: "--ros" requires a non-empty option argument.'
            ;;
        -r|--run)
            if [ "$2" ]; then
                shift
                USER_COMMAND=" $@ "
            else
                die 'ERROR: "--run" requires a non-empty option argument.'
            fi
            ;;
        --)
            shift
            break
            ;;
        -?*)
            printf 'WARN: Unknown option (ignored): %s\n' "$1" >&2
            ;;
        *)   # default case: No more options, so break out of the loop.
            break
    esac

    shift
done

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

V4L2_DEVICES=" --device /dev/video0"
I2C_DEVICES=" --device /dev/i2c-7"

# check for display
DISPLAY_DEVICE=""

if [ -n "$DISPLAY" ]; then
	sudo xhost +si:localuser:root
	DISPLAY_DEVICE=" -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v /tmp/.docker.xauth:/tmp/.docker.xauth -e XAUTHORITY=/tmp/.docker.xauth " 
fi


DATA_VOLUME=" -v /home/orin/oribot:/oribot"

# print configuration
print_var() 
{
	if [ -n "${!1}" ]; then                                                # reference var by name - https://stackoverflow.com/a/47768983
		local trimmed="$(echo -e "${!1}" | sed -e 's/^[[:space:]]*//')"   # remove leading whitespace - https://stackoverflow.com/a/3232433    
		printf '%-17s %s\n' "$1:" "$trimmed"                              # justify prefix - https://unix.stackexchange.com/a/354094
	fi
}

print_var "CONTAINER_IMAGE"
print_var "CONTAINER_NAME"
print_var "ROS_DISTRO"
print_var "DATA_VOLUME"
print_var "DEV_VOLUME"
print_var "USER_VOLUME"
print_var "USER_COMMAND"
print_var "V4L2_DEVICES"
print_var "I2C_DEVICES"
print_var "DISPLAY_DEVICE"

sudo at /proc/device-tree/model > /tmp/nv_jetson_model

	sudo docker run --runtime nvidia -it --rm \
		--network host \
        --name $CONTAINER_NAME \
		-v /tmp/argus_socket:/tmp/argus_socket \
		-v /etc/enctune.conf:/etc/enctune.conf \
		-v /etc/nv_tegra_release:/etc/nv_tegra_release \
		-v /tmp/nv_jetson_model:/tmp/nv_jetson_model \
		$DISPLAY_DEVICE $V4L2_DEVICES $I2C_DEVICES \
		$DATA_VOLUME $USER_VOLUME $DEV_VOLUME \
		$CONTAINER_IMAGE /bin/bash
