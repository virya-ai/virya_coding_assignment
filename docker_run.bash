# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.
## Build the image first
### docker build -t r2_path_planning .
## then run this script
xhost local:root


XAUTH=/tmp/.docker.xauth

sudo docker run \
    -v /etc/localtime:/etc/localtime:ro \
    -v /etc/timezone:/etc/timezone:ro \
    -it \
    --privileged \
    --name=virya_test_container \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net="host" \
    --ipc="host" \
    --pid="host" \
    --privileged \
    virya_ros2_humble \
    bash


echo "Done."
    
