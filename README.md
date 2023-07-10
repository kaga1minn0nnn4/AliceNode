# AliceNode

## Overview
AliceNode is main node for object search robot.

## Download and Build
    cd ~/catkin_ws/src
    git clone --recursive https://github.com/toida2003/AliceNode.git
    cd ~/catkin_ws
    catkin build
    source ~/catkin_ws/devel/setup.bash

## Using
    cp <your-slammap>.pgm ~/catkin_ws/src/AliceNode/map/
    cp <your-slammap>.yaml ~/catkin_ws/src/AliceNode/map/
    roslaunch alicenode main.launch

## Dependencies
- rplidar_ros
- cartographer
- realsense-ros
- [convert-twistmsg](https://github.com/toida2003/convert_twistmsg)
- [FangFSM](https://github.com/toida2003/FangFSM)
