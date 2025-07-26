#!/bin/bash

# X11 の DISPLAY 設定
if [ -n "$DISPLAY" ]; then
    xauth list $DISPLAY >/dev/null 2>&1 || xauth add $DISPLAY . $(xxd -l 16 -p /dev/urandom)
fi

# ROS の setup
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash

exec "$@"

