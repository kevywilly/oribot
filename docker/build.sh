#!/usr/bin/env bash
#./scripts/docker_build_ros.sh --distro iron --package desktop --with-pytorch
sudo docker build --network=host -t kevywilly/orin:1.0 --build-arg CACHEBUST=$(date +%s) -f Dockerfile .
