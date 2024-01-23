#!/bin/bash
TARGET_IP=192.168.1.31 # add your host PC IP here
TARGET_PORT=5600 # port configured for QGroundControl
source install/setup.bash
gst-launch-1.0 --gst-plugin-path=install/gst_bridge/lib/gst_bridge/ rosimagesrc \
ros-topic=/bright_spot_detector ! video/x-raw,format=BGR ! videoconvert ! \
x264enc bitrate=2100 tune=zerolatency speed-preset=ultrafast ! \
video/x-h264,stream-format=byte-stream ! rtph264pay ! udpsink \
host=$TARGET_IP port=$TARGET_PORT sync=false