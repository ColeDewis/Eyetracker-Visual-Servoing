#!/usr/bin/env python

import rospkg
import numpy as np
from roboticstoolbox.robot.Robot import Robot


class KinovaGen3Camera(Robot):
    """
    Class that imports a KinovaGen3 URDF model with the camera links
    """

    def __init__(self):

        rospack = rospkg.RosPack()
        path = rospack.get_path("simulator")
        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "models/robots/gen3_camera.xacro", tld=path
        )

        super().__init__(
            links,
            name=name,
            manufacturer="Kinova",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        self.qr = np.array([np.pi, -0.3, 0, -1.6, 0, -1.0, np.pi / 2])
        self.qz = np.zeros(7)

        self.qlim = np.array(
            [
                [-3.14, -2.41, -3.14, -2.66, -3.14, -2.23, -3.14],
                [3.14, 2.41, 3.14, 2.66, 3.14, 2.23, 3.14],
            ]
        )
        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)
