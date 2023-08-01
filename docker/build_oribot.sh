#!/usr/bin/env bash
#./scripts/docker_build_ros.sh --distro iron --package desktop --with-pytorch
sudo docker build --network=host -t kevywilly/oribot:1.0 --build-arg CACHEBUST=$(date +%s) -f Dockerfile.oribot .
sudo docker tag kevywilly/oribot:1.0 kevywilly/oribot:latest