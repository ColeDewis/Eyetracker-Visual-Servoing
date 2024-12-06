import sys

import rospy
import cv2
import tf2_ros
import tf_conversions
import cv_bridge

import numpy as np
from numpy.linalg import pinv, norm

from kortex_bringup import KinovaGen3, SimulatedGen3, KinovaUtil

from simulator.kinova_gen3_camera import KinovaGen3Camera

from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import TransformStamped, Transform
from std_msgs.msg import String

from custom_msgs.msg import (
    Point2D,
)


class VS_PFC:
    """Visual Servoing PFC Node."""

    def __init__(self):
        """Initialize eye following, setting up the required ROS connections."""

        rospy.init_node("vs_eye_follow")
        rospy.loginfo("Starting Visual Servo Eye Following Node")

        self.is_sim = rospy.get_param("~sim", True)

        self.sim_robot = KinovaGen3Camera()
        if self.is_sim:
            self.kinova = SimulatedGen3()
        else:
            self.kinova = KinovaGen3()

        self.kinova_util = KinovaUtil()

        self.robot_kin = KinovaGen3Camera()

        self.bridge = cv_bridge.CvBridge()

        # state variables
        self.last_target = None
        self.last_depth_array = None
        self.last_eye_time = rospy.Time(0)
        self.last_joints = np.zeros(7)

        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # --- Subscribers ---
        self.eye_sub = rospy.Subscriber(
            "/eyetracker/vs_target",
            Point2D,
            self.target_callback,
        )
        # self.img_sub = rospy.Subscriber(
        #     f"/camera/color/image_raw",
        #     Image,
        #     self.visualization_cb,
        #     queue_size=1,
        # )
        self.depth_sub = rospy.Subscriber(
            f"/camera/aligned_depth_to_color/image_raw",
            Image,
            self.depth_img_cb,
        )
        self.joint_sub = rospy.Subscriber(
            "/my_gen3/joint_states", JointState, self.joint_cb
        )

        rospy.sleep(0.5)
        rospy.loginfo("VS is ready to run!")
        self.visual_servo_loop()

    def joint_cb(self, msg: JointState):
        self.last_joints = msg.position[:7]

    def target_callback(self, msg: Point2D):
        self.last_target = np.array([msg.x, msg.y])
        self.last_eye_time = rospy.get_rostime()

    def move_vel(self, velocities):
        """Send the given twist to the kinova

        Args:
            velocities (list): list of twist, 3 linear in m/s, 3 angular in deg/s
        """
        # velocities[3:] = np.rad2deg(velocities[3:])  # service wants deg/s not rad/s
        # self.kinova.send_twist(vels=velocities, duration_ms=0)

        self.kinova.send_twist_topic(vels=velocities, duration_ms=0)

    def get_adjoint_matrix(self, rot, trans):
        """Given a rotation matrix and translation vector, returns the adjoint representation of the transformation in R3

        Args:
            rot (list): 3x3 rotation matrix from camera to eef
            trans (list): vector in R3 representing the translation from camera to eef

        Returns:
            matrix: 6x6 numpy matrix representing the adjoint matrix
        """
        skew_t = np.array(
            [
                [0, -trans[2], trans[1]],
                [trans[2], 0, -trans[0]],
                [-trans[1], trans[0], 0],
            ]
        )
        upper = np.hstack([rot, skew_t @ rot])
        lower = np.hstack([np.zeros((3, 3)), rot])
        adjoint_mat = np.vstack([upper, lower])

        return adjoint_mat

    def depth_img_cb(self, data: Image):
        """Callback for a new frame of depth data

        Args:
            data (Image): image message containing depth data.
        """
        depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)
        self.last_depth_array = depth_array / 1000  # convert to meters

    def generate_interaction_matrix(self, desired_points) -> list:
        """Generates the interaction matrix for velocity control for given image points and camera transformation.

        Args:
            desired_points (list): list of x,y,z desired image points + estimated depths
            camera_idx (int): index of camera these points are for - needed to get the transform from camera to tool link.

        Returns:
            list: interaction matrix
        """
        # get transform from camera to base
        try:
            cam_frame = "camera_color_frame"
            eef_transform: TransformStamped = self.tf_buffer.lookup_transform(
                "base_link",
                cam_frame,
                rospy.Time(0),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn(f"Failed to lookup transform for L: {e}")
            return

        eef_transform: Transform = eef_transform.transform
        translation = np.array(
            [
                eef_transform.translation.x,
                eef_transform.translation.y,
                eef_transform.translation.z,
            ]
        )
        q = np.array(
            [
                eef_transform.rotation.x,
                eef_transform.rotation.y,
                eef_transform.rotation.z,
                eef_transform.rotation.w,
            ]
        )
        rot = tf_conversions.transformations.quaternion_matrix(q)[:3, :3]
        adj_mat = self.get_adjoint_matrix(rot, translation)

        L = None
        for point in desired_points:
            x, y, Z = point
            f = 640

            part_L = np.array(
                [
                    [-f / Z, 0, x / Z, x * y / f, -(f + x * x / f), y],
                    [0, -f / Z, y / Z, f + y * y / f, -x * y / f, -x],
                ]
            )

            # transform interaction matrix to base. see https://inria.hal.science/inria-00350283/document
            part_L = -part_L @ adj_mat
            if L is None:
                L = part_L
            else:
                L = np.row_stack([L, part_L])

        return L

    def visual_servo_loop(
        self,
        lambda_step=1.5,
        alpha_1=0.75,
        alpha_2=0.25,
        beta_1=0.25,
        beta_2=0.75,
        rate=30,
        use_depth=True,
        max_it=np.inf,
    ):
        """Path Following Controller Loop

        Args:
            lambda_step (float, optional): step gain for positional control. Defaults to 1.0.
            alpha_1 (float, optional): alpha interaction matrix weight. Defaults to 0.75.
            alpha_2 (float, optional): alpha interaction matrix weight. Defaults to 0.25.
            beta_1 (float, optional): beta interaction matrix weight. Defaults to 0.25.
            beta_2 (float, optional): beta interaction matrix weight. Defaults to 0.75.
            decay (float, optional): decay of velocity term weight as error increases. Defaults to 25.0.
            use_depth (bool, optional): whether or not to use rgb-d depth data. Defaults to True.
            max_it (int, optional): maximum allowed iterations. Defaults to infinity.

        Returns:
            tuple: total error, positional error, velocity error, velocity data, and image poses
        """
        if use_depth:
            while self.last_depth_array is None:
                rospy.sleep(0.1)

        while self.last_target is None:
            rospy.sleep(0.1)

        # TODO: need to publish the frame we need, need to test in sim.

        # iterate:
        RATE = rate
        LAMBDA = lambda_step
        ALPHA_1 = alpha_1
        ALPHA_2 = alpha_2
        BETA_1 = beta_1
        BETA_2 = beta_2
        r = rospy.Rate(RATE)
        it = 0

        error_pos = []
        targets = []
        last_depth = 0.5

        pose = np.array([320, 240])
        while it < max_it:
            start = rospy.get_rostime().to_sec()

            error_p = np.hstack([pose - self.last_target, pose - self.last_target])

            # interaction for the end effector
            depth = self.__get_pixel_depth(pose) if use_depth else 0.5
            depth = depth if depth != 0 else last_depth
            last_depth = depth

            L_bar = self.generate_interaction_matrix(
                np.array([[pose[0], pose[1], depth]])
            )

            L_star = self.generate_interaction_matrix(
                np.array([[self.last_target[0], self.last_target[1], depth]])
            )

            error_p[:2] = (ALPHA_1 + ALPHA_2) * error_p[:2]
            error_p[2:] = (BETA_1 + BETA_2) * error_p[2:]

            L = np.zeros((4, 6))
            L[:2, :] = ALPHA_1 * L_bar + ALPHA_2 * L_star
            L[2:, :] = BETA_1 * L_bar + BETA_2 * L_star
            L_inv = pinv(L)

            vels_p = -LAMBDA * L_inv @ error_p.T

            if abs(rospy.get_rostime().to_sec() - self.last_eye_time.to_sec()) < 0.1:
                self.move_vel(vels_p)
            else:
                self.move_vel(np.zeros(6))

            error_pos.append(error_p[:2])
            targets.append(self.last_target)

            r.sleep()
            rospy.loginfo(f"Time: {rospy.get_rostime().to_sec() - start}, it: {it}")
            it += 1

        # done, stop moving
        self.move_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # reset tracking
        self.track_reset_pub.publish(String(data="vspfc"))

        return error_pos, targets

    def __get_pixel_depth(self, pos):
        """Gets the most recent depth for the given position

        Args:
            pos (list): u, v image coordinates
        """
        return self.last_depth_array[int(pos[1]), int(pos[0])]


def main(args):
    rospy.loginfo("Starting vs node...")
    node = VS_PFC()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down vs node...")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
