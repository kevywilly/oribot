<<<<<<< HEAD
#!/usr/bin/env bash
#./scripts/docker_build_ros.sh --distro iron --package desktop --with-pytorch
sudo docker build --network=host -t kevywilly/orin:1.0 --build-arg CACHEBUST=$(date +%s) -f Dockerfile .
=======
#!/bin/bash

die () {
    echo >&2 "$@"
    exit 1
}
[ "$#" -eq 1 ] || die "1 argument required, $# provided"
echo $1 | grep -E -q '^[0-9.]+$' || die "Numeric argument required, $1 provided"

BASE_TAG="kevywilly/orin-ros2-humble"
TAG="$BASE_TAG:$1"
LATEST="$BASE_TAG:latest"

echo $TAG
echo $LATEST

sudo docker build --network=host -t $TAG -f Dockerfile .
sudo docker tag $TAG $LATEST
>>>>>>> v3.0
