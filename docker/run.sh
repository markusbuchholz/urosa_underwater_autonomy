#!/usr/bin/env bash

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

#Adjust these paths
local_auto="/home/markus/underwater/robot_autonomy/"
local_src="/home/markus/underwater/robot_autonomy/src/"

# Mount the host binary and related directories
host_ollama_bin="/usr/local/bin/ollama"
host_ollama_lib="/usr/local/lib/ollama"
host_ollama_dir="/usr/local/ollama"
host_ollama_models="/usr/share/ollama/.ollama/models" 

docker run -it \
    --rm \
    --name autonomy \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev/input:/dev/input" \
    -v "$local_auto:/home/robot_autonomy" \
    -v "$local_src:/home/rosuser/colcon_ws/src" \
    -v "$host_ollama_bin:/usr/local/bin/ollama:ro" \
    -v "$host_ollama_lib:/usr/local/lib/ollama:ro" \
    -v "$host_ollama_dir:/usr/local/ollama:ro" \
    -v "$host_ollama_models:/usr/share/ollama/.ollama/models" \
    --device /dev/video0:/dev/video0 \
    --privileged \
    --security-opt seccomp=unconfined \
    --network host \
    --gpus all \
    autonomy:latest
