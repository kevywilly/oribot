#!/usr/bin/env bash
#./scripts/docker_build_ros.sh --distro iron --package desktop --with-pytorch
sudo docker build --network=host -t kevywilly/orin-ros2-iron:1.0 -f Dockerfile .
