#!/bin/bash

# Get the absolute path of the directory where the script resides
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Allow X server access
xhost +local:root

# Run the Docker container
docker run -it \
  --net=host \
  --ipc=host \
  --privileged \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="${XAUTHORITY}:/root/.Xauthority" \
  --volume="${SCRIPT_DIR}:/root/ros2_ws/src/:rw" \
  --entrypoint /bin/bash \
  yasmin
