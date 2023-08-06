#!/bin/bash

${ORIBOT_ROOT}/docker/scripts/run.sh -v /home/orin/oribot:/oribot -c kevywilly/oribot:latest -n oribot.service -r /bin/bash