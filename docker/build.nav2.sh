#!/bin/bash

die () {
    echo >&2 "$@"
    exit 1
}
[ "$#" -eq 1 ] || die "1 argument required, $# provided"
echo $1 | grep -E -q '^[0-9.]+$' || die "Numeric argument required, $1 provided"

BASE_TAG="kevywilly/orin-ros2-humble-nav2"
TAG="$BASE_TAG:$1"
LATEST="$BASE_TAG:latest"

echo $TAG
echo $LATEST

sudo docker build --network=host -t $TAG -f Dockerfile .
sudo docker tag $TAG $LATEST
