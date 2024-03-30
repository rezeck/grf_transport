docker run --rm -it \
    --privileged \
    --net=host \
    --gpus all \
    -e NVIDIA_VISIBLE_DEVICES=0 \
    --env=DISPLAY \
    --env=NVIDIA_DRIVER_CAPABILITIES=all \
    --env=QT_X11_NO_MITSHM=1 \
    --volume=$HOME/.Xauthority:/root/.Xauthority:rw \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume=$PWD:/catkin_ws/src/grf_transport/:rw \
    grf_swarm_tranport:v1.0 \
    /bin/bash -c '. /opt/ros/noetic/setup.bash && . /catkin_ws/devel/setup.bash && roslaunch grf_transport demo.launch'