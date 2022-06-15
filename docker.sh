IMAGE=qazmichaelgw/intent_map
ROOT=$1

# allow X server
xhost +local:*

docker run --privileged -ti --net=host --rm --ipc=host \
           -e DISPLAY=$DISPLAY \
           -e QT_X11_NO_MITSHM=1 \
           -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
           -v $ROOT:/mnt/ \
           $IMAGE \
           bash
