#!/bin/bash
  
  
 docker run -it --privileged \
  --env=LOCAL_USER_ID="$(id -u)" \
  -v ~/ws/ws2:/root/ws:rw \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  --network host \
  --device=/dev/dri \
  --runtime=nvidia --gpus all\
  --name=perception_docker nvidia/cuda:11.8.0-base-ubuntu22.04 bash 
