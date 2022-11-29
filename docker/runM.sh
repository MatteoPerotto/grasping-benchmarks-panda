xhost +

docker run \
        -it \
        --name=ros_yarp_gpd  \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --gpus=all \
        --runtime=nvidia \
        --network=host \
        --privileged \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
	matteoperotto/benchmark_gpd:latest 	        
xhost -
