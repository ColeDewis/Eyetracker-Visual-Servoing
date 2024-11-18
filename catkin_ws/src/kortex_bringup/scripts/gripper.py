#!/usr/bin/python3

import rospy
import sys

from kortex_bringup import KinovaGen3, SimulatedGen3

if __name__ == "__main__":
    pos = float(sys.argv[1])

    # Init ros node
    rospy.init_node("kortex_gripper_cmd", anonymous=False)

    # Robot node
    gen3 = KinovaGen3()
    gen3.send_gripper_command(pos)
    print("Done. Joints:", gen3.position)
