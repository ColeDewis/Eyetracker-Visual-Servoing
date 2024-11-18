#!/bin/sh
ROSBAG_NAME="/home/user/landmine/bags/robot/$(eval date +"%F")_$(eval date +"%T.%3N")_robot.bag"
rosbag record -O $ROSBAG_NAME /my_gen3/joint_states /camera/color/image_raw /camera/aligned_depth_to_color/image_raw