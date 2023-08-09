#!/bin/bash

die () {
    echo >&2 "$@"
    exit 1
}
[ "$#" -eq 1 ] || die "Version Argument is required, $# provided"
echo $1 | grep -E -q '^[0-9.]+$' || die "Numeric version argument required, $1 provided"

BASE_TAG="kevywilly/oribot-isaac"
TAG="$BASE_TAG:$1"
LATEST="$BASE_TAG:latest"

echo $TAG
echo $LATEST

sudo docker build --network=host -t $TAG --build-arg CACHEBUST=$(date +%s) -f Dockerfile.oribot.isaac .
docker tag $TAG $LATEST
