#!/bin/bash

# Author: مهدي بلال

docker run -it --rm \
  --privileged \
  --shm-size=2g \
  --pid=host \
  --cap-add SYS_PTRACE \
  -e TERM=xterm-color \
  -v /dev/shm:/dev/shm \
  --env DISPLAY=${DISPLAY} \
  -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
  --name 3d_simulator \
  3d_simulator /bin/bash