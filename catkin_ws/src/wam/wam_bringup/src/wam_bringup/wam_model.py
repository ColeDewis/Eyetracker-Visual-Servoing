#!/usr/bin/env python3

import rospkg
import numpy as np
from roboticstoolbox.robot.Robot import Robot


class WamModel(Robot):
    """
    Class that imports a WAM URDF model
    """

    def __init__(self):
        # TODO: add DOF arg?
        rospack = rospkg.RosPack()
        path = rospack.get_path("wam_bringup")
        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "models/wam7dof/wam7dof_hand.urdf", tld=path
        )

        super().__init__(
            links,
            name=name,
            manufacturer="Barrett",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        # NOTE: not sure if i need to define these here, I think I don't?
        # self.qr = np.array([np.pi, -0.3, 0, -1.6, 0, -1.0, np.pi / 2])
        # self.qz = np.zeros(7)

        # TODO find
        # self.qlim = np.array(
        #     [
        #         [-3.14, -2.41, -3.14, -2.66, -3.14, -2.23, -3.14],
        #         [3.14, 2.41, 3.14, 2.66, 3.14, 2.23, 3.14],
        #     ]
        # )
        # self.addconfiguration("qr", self.qr)
        # self.addconfiguration("qz", self.qz)

if __name__ == "__main__":
    robot = WamModel()
    print(robot.hierarchy())