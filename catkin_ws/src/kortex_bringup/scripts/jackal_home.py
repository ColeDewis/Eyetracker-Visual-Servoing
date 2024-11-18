#!/usr/bin/python3

import numpy as np
import rospy

from kortex_bringup import KinovaGen3

if __name__ == "__main__":
    # Init ros node
    rospy.init_node("send_gen3_home", anonymous=False)

    # Robot node
    gen3 = KinovaGen3()

    print()
    angles = np.array(
        [
            0.0003903112899049132,
            -2.235807624695858,
            -3.107286611375719,
            -2.5612363225236163,
            -0.06361759211981344,
            -0.8495276150787738,
            1.5669007879103831,
        ]
    )
    success = gen3.send_joint_angles(angles)

    print("Done. Joints:", gen3.position)
