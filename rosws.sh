#!/usr/bin/bash
IMAGE_NAME=ros2-irs-rascl-gruppe16
CONTAINER_NAME=${CONTAINER_NAME:-rascl-gruppe16}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-16}

set -e

# Check if the container is already running
if ${NO_ATTACH:-false} || ! docker ps --format "{{.Names}}" | grep -q "$CONTAINER_NAME"; then
  if ${REBUILD:-false}; then
    echo "Rebuilding Container without cache"
    docker buildx build --network host --no-cache -t $IMAGE_NAME .
  elif ${SOFT_REBUILD:-false} || ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo "Building Container"
    docker buildx build --network host -t $IMAGE_NAME .
  else
    echo "Container build has been skipped since an image already exists."
    echo "If the container has been changed, you may want to rebuild it"
    echo "by passing the environment variable REBUILD=true."
  fi
  echo "Starting Container ..."
  mkdir -p .devcontainer/
  touch .devcontainer/.bash_history

  # Detect WSL and set X11 socket mount accordingly
  if grep -qiE "microsoft|wsl" /proc/version; then
    X11_MOUNT="-v /mnt/wslg/.X11-unix:/tmp/.X11-unix"
  else
    X11_MOUNT="-v /tmp/.X11-unix:/tmp/.X11-unix"
  fi

  docker run -it --rm \
    --name ${CONTAINER_NAME} \
    --privileged \
    --network=host \
    --cap-add CAP_NET_RAW \
    --cap-add CAP_NET_ADMIN \
    --cap-add CAP_IPC_LOCK \
    --cap-add CAP_SYS_NICE \
    -v ./:/root/ws \
    -v ./.devcontainer/.bash_history:/root/.bash_history \
    ${X11_MOUNT} \
    -v ${XDG_RUNTIME_DIR}/${WAYLAND_DISPLAY}:/run/user/$(id -u)/${WAYLAND_DISPLAY} \
    -v ${HOME}/.Xauthority:/root/.Xauthority \
    -e XAUTHORITY=/root/.Xauthority \
    -e DISPLAY \
    -e WAYLAND_DISPLAY \
    -e XDG_RUNTIME_DIR=/run/user/$(id -u) \
    -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
    -e "TERM=xterm-256color" \
    -e "QT_X11_NO_MITSHM=1" \
    -e LIBGL_ALWAYS_SOFTWARE=0 \
    -e MESA_GL_VERSION_OVERRIDE=3.3 \
    -u ${localEnv:UID}:${localEnv:GID} \
    --log-driver=none \
    --device /dev/cpu_dma_latency \
    ${IMAGE_NAME}
else
  echo "Attaching to running container ..."
  echo "$INFO"
  docker exec -it ${CONTAINER_NAME} bash
fi

echo "Good bye!"
