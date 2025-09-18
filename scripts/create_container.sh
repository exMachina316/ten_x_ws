#!/bin/bash

./stop.sh

docker run -it -d --privileged --net=host \
--name ten_x_dev \
--env="QT_X11_NO_MITSHM=1"  \
--env="DISPLAY"  \
-v $PWD/../workspace:/root/ten_x_ws/src \
-v /dev:/dev \
ten-x-dev-img:latest
