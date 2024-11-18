#!/bin/sh
ROSBAG_NAME="/home/user/landmine/bags/sim/$(eval date +"%F")_$(eval date +"%T.%3N")_simulated.bag"
rosbag record -O $ROSBAG_NAME /my_gen3/joint_states /cameras/cam0/aligned_depth /cameras/cam0