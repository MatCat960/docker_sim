#!/bin/bash

# Author: مهدي بلال

docker run -it \
  --privileged \
  --shm-size=2g \
  --pid=host \
  --net=host \
  --cap-add SYS_PTRACE \
  -e TERM=xterm-color \
  -v /dev/shm:/dev/shm \
  --env DISPLAY=${DISPLAY} \
  -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
  --name sim_docker \
  8d78ee302367 /bin/bash
