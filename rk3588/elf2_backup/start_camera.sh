#!/bin/bash
# HP60C Trash Detector - One-click startup
export DISPLAY=:0

# WDT feed
echo elf | sudo -S bash -c "while true; do echo 0 > /dev/watchdog0; sleep 10; done" &

# Kill old
pkill -9 -f rqt 2>/dev/null
pkill -9 -f trash_detector 2>/dev/null
sleep 1

# SDK Camera
source /opt/ros/humble/setup.bash
source /home/elf/hp60c_ws/install/setup.bash
nohup ros2 launch ascamera hp60c.launch.py > /dev/null 2>&1 &
echo "[1/3] Camera starting..."

# Wait for camera
for i in 1 2 3 4 5 6 7 8; do
    n=
    [ "$n" -ge 3 ] && break
    sleep 1
done
echo "[2/3] Detector starting..."
source /home/elf/hp60c_ws/install/setup.bash
nohup python3 /home/elf/trash_detector.py > /dev/null 2>&1 &
sleep 3

# LCD display
xhost + 2>/dev/null
ros2 run rqt_image_view rqt_image_view /trash_detector/result &
sleep 5
wmctrl -r rqt -b toggle,fullscreen 2>/dev/null
echo "[3/3] Done - LCD should show detection"
