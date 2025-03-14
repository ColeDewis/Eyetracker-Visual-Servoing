import numpy as np
import rospy
from wam_bringup import WAM

if __name__ == "__main__":
    rospy.init_node("wam_test_node", anonymous=False)
    wam = WAM()
    zero = np.zeros(7)
    wam.send_joint_velocities(zero)

    small = np.array([0.1, 0, 0, 0, 0, 0, 0])
    wam.send_joint_velocities(small)
    rospy.sleep(0.1)
    wam.send_joint_velocities(zero)

    rospy.loginfo(f"Done. Joint angles: {wam.joints}")
