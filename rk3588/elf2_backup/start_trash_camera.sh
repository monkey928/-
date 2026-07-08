#!/bin/bash
# Trash Detector Camera Viewer Launcher

source /opt/ros/humble/setup.bash
source /home/elf/hp60c_ws/install/setup.bash

# Kill old viewer only
pkill -f "ros2 run rqt_image_view" 2>/dev/null
sleep 1

# Check if camera is running, start if not
if ! ros2 node list 2>/dev/null | grep -q ascamera_hp60c; then
    pkill -f trash_detector 2>/dev/null
    sleep 1
    nohup ros2 launch ascamera hp60c.launch.py > /tmp/camera.log 2>&1 &
    sleep 8
    nohup python3 /home/elf/trash_detector.py --ros-args -p camera_ns:=ascamera_hp60c > /tmp/trash.log 2>&1 &
    sleep 3
else
    echo "Camera already running"
    if ! ros2 node list 2>/dev/null | grep -q trash_detector; then
        nohup python3 /home/elf/trash_detector.py --ros-args -p camera_ns:=ascamera_hp60c > /tmp/trash.log 2>&1 &
        sleep 3
    fi
fi

ros2 run rqt_image_view rqt_image_view /trash_detector/result