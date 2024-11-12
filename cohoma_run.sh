#!/bin/bash

docker run -it --privileged \
  --env=LOCAL_USER_ID="$(id -u)" \
  -v ~/ws/ws1:/root/ws:rw \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -e DISPLAY=:0 \
  --network host \
  --name=cohoma_humble ubuntu:22.04 bash

