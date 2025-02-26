import sys

import rospy
import cv2
import tf2_ros
import tf_conversions
import cv_bridge

import numpy as np
from numpy.linalg import pinv, norm
from typing import Tuple

from kortex_bringup import KinovaGen3, SimulatedGen3, KinovaUtil

from simulator.kinova_gen3_camera import KinovaGen3Camera

from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import TransformStamped, Transform
from std_msgs.msg import String

from custom_msgs.msg import Point2D, MaskArray


class UVS:
    """Visual Servoing."""

    def __init__(self):
        """Initialize eye following, setting up the required ROS connections."""

        rospy.init_node("vs_eye_follow", disable_signals=True)
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
        self.pose = None
        self.last_im = None
        self.last_target = None
        self.last_mask = None
        self.last_depth_array = None
        self.last_eye_time = rospy.Time(0)
        self.last_joints = np.zeros(7)
        self.last_sign = "plus"

        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # --- Publishers ---
        self.im_pub = rospy.Publisher("/visual_servo/debug", Image, queue_size=10)
        self.pca_pub = rospy.Publisher("/visual_servo/pca_debug", Image, queue_size=10)

        # --- Subscribers ---
        self.eye_sub = rospy.Subscriber(
            "/eyetracker/vs_target",
            Point2D,
            self.eye_target_callback,
        )
        self.sam_mask_sub = rospy.Subscriber(
            "/sam2/masks", MaskArray, self.mask_callback
        )
        self.img_sub = rospy.Subscriber(
            f"/camera/color/image_raw",
            Image,
            self.visualization_cb,
            queue_size=1,
        )
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

    def visualization_cb(self, msg: Image):
        im = self.bridge.imgmsg_to_cv2(msg, "rgb8")

        if self.pose is not None:
            for i, pt in enumerate(self.pose):
                im = cv2.circle(
                    im, (int(pt[0]), int(pt[1])), 5, (0, 0, (i + 1) * 60), -1
                )

        if self.last_target is not None:
            for i, pt in enumerate(self.last_target):
                im = cv2.circle(
                    im, (int(pt[0]), int(pt[1])), 5, ((i + 1) * 60, 0, 0), -1
                )

        self.last_im = im
        self.im_pub.publish(self.bridge.cv2_to_imgmsg(im, "rgb8"))

    def joint_cb(self, msg: JointState):
        self.last_joints = msg.position[:7]

    def eye_target_callback(self, msg: Point2D):
        self.last_target = np.array([msg.x, msg.y])
        self.last_eye_time = rospy.get_rostime()

    def mask_callback(self, msg: MaskArray):
        """Callback for masks from SAM2

        Args:
            msg (MaskArray): message containing array of all masks
        """

        # for now just always grab mask 0
        mask = msg.masks[0]
        self.last_mask = self.bridge.imgmsg_to_cv2(mask, "8UC1") // 255

    def move_vel(self, velocities):
        """Send the given twist to the kinova

        Args:
            velocities (list): list of twist, 3 linear in m/s, 3 angular in deg/s
        """
        # velocities[3:] = np.rad2deg(velocities[3:])  # service wants deg/s not rad/s
        # self.kinova.send_twist(vels=velocities, duration_ms=0)

        self.kinova.send_twist_topic(vels=velocities, duration_ms=0)

    def move_joint_vel(self, velocities):
        self.kinova.send_joint_velocities(velocities)

    def depth_img_cb(self, data: Image):
        """Callback for a new frame of depth data

        Args:
            data (Image): image message containing depth data.
        """
        depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)
        self.last_depth_array = depth_array / 1000  # convert to meters

    def drawAxis(self, img, p_, q_, colour, scale):
        # https://docs.opencv.org/4.x/d1/dee/tutorial_introduction_to_pca.html
        p = list(p_)
        q = list(q_)

        angle = np.arctan2(p[1] - q[1], p[0] - q[0])  # angle in radians
        hypotenuse = np.sqrt(
            (p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0])
        )

        # Here we lengthen the arrow by a factor of scale
        q[0] = p[0] - scale * hypotenuse * np.cos(angle)
        q[1] = p[1] - scale * hypotenuse * np.sin(angle)
        cv2.line(
            img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA
        )

        # create the arrow hooks
        p[0] = q[0] + 9 * np.cos(angle + np.pi / 4)
        p[1] = q[1] + 9 * np.sin(angle + np.pi / 4)
        cv2.line(
            img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA
        )

        p[0] = q[0] + 9 * np.cos(angle - np.pi / 4)
        p[1] = q[1] + 9 * np.sin(angle - np.pi / 4)
        cv2.line(
            img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA
        )

    def init_jacobian(self, error_dim: int, is_joint_vel: bool = False, n_joints: int = 0):
        """Initializes jacobian given dimension of error

        Args:
            error_dim (int): dimension of error vector
            is_joint_vel (bool): true for joint velocity; false for cartesian velocity. Defaults to false
            n_joints (int): if is_joint_vel is True, number of joints to use; defaults to 0, so must be set.
        """
        if is_joint_vel:
            jacob =  np.zeros((error_dim, n_joints))
            dim = n_joints
            move_func = self.move_joint_vel
        else:
            jacob = np.zeros((error_dim, 6))
            dim = 6
            move_func = self.move_vel

        WAIT_TIME = 0.5
        VEL_SCALE = 0.1
        vels = np.identity(dim) * VEL_SCALE
        for i in range(6):
            # IDEA: basis updates where we move at some velocity and observe feature velocity
            vel = vels[i, :]
            
            move_func(vel)
            init_pts = self.mask2pcaconstraints(self.last_mask, 1.5)
            rospy.sleep(WAIT_TIME)
            rospy.wait_for_message("/sam2/masks", Image)

            # get the error change
            after_pts = self.mask2pcaconstraints(self.last_mask, 1.5)

            move_func(-vel)
            rospy.sleep(WAIT_TIME)

            jacob[:, i] = ((after_pts - init_pts) / (WAIT_TIME)).flatten() # NOTE maybe wrong uhhh
        
        self.move_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        return jacob, pinv(jacob)

    def broyden_update(self, jacobian: np.ndarray, vel: np.ndarray, features_vel: np.ndarray, alpha_step: float, joint_thresh: float) -> Tuple: 
        """Updates the Jacobian using a broyden update

        Args:
            jacobian (np.Array): current jacobian matrix
            features_vel (np.Array): velocity of feature errors
            alpha_step (float): step size for update
            joint_thresh (float): threshold the joints must have moved to allow an update

        Returns:
            tuple(np.Array, np.Array): new jacobian, (pseudo)-inverse of new jacobian
        """
        # mapping b/w velocity (or joint velocity) and feature velocity
        del_theta = vel
        del_e = features_vel
        
        # if joints havent moved enough, don't update on this iteration
        if norm(del_theta) < joint_thresh: 
            rospy.loginfo(f'No Update')
            return jacobian, pinv(jacobian)
        
        # broyden update
        numerator = del_e - jacobian @ del_theta
        rospy.loginfo(f"{numerator.shape}, {del_theta.shape}, {del_e.shape}")
        update = numerator.reshape(-1, 1) @ del_theta.reshape(1, -1) / (del_theta.reshape(1, -1) @ del_theta)
        
        # update jacobian 
        jacobian = jacobian + alpha_step * update
        
        return jacobian, pinv(jacobian)

    def mask2pcaconstraints(
        self,
        mask,
        expand_scale: float,
        scale_maj: float = 0.02,
        scale_min: float = 0.02,
    ) -> np.ndarray:
        """Convert a mask to 4 points via PCA

        Args:
            mask (list): mask image
            expand_scale (float): scale for how much to expand points from the centroid based on minor axis
            scale_maj (float, optional): scale for major axis eigenvectors. Defaults to 0.02.
            scale_min (float, optional): scale for minor axis eigenvectors. Defaults to 0.02.

        Returns:
            np.ndarray: list of points, starting at top left and going clockwise
        """

        data_points = cv2.findNonZero(mask).sum(axis=1).astype(np.float32)

        mean = np.empty((0))
        mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_points, mean)
        cntr = (int(mean[0, 0]), int(mean[0, 1]))

        # major
        maj_ax = (
            cntr[0] + scale_maj * eigenvectors[0, 0] * eigenvalues[0, 0],
            cntr[1] + scale_maj * eigenvectors[0, 1] * eigenvalues[0, 0],
        )

        # minor
        min_ax = (
            cntr[0] - scale_min * eigenvectors[1, 0] * eigenvalues[1, 0],
            cntr[1] - scale_min * eigenvectors[1, 1] * eigenvalues[1, 0],
        )
        angle = np.arctan2(eigenvectors[0, 1], eigenvectors[0, 0])  # radians
        # angle is to maj ax, "Up" negative "Down" positive

        cntr = np.array(cntr)
        min_ax = np.array(min_ax)
        min_vec = expand_scale * (min_ax - cntr)
        min_orth_vec = np.array([-min_vec[1], min_vec[0]])

        sign = "plus" if min_vec[0] > 0 else "minus"
        # rospy.loginfo(f"{min_vec}, {sign}, {self.last_sign}")
        if min_vec[0] > 0.1 and sign != self.last_sign:
            min_vec *= -1
            sign = "plus" if sign == "minus" else "minus"
        self.last_sign = sign

        # debug visualization for PCA
        # mask_im = cv2.cvtColor(mask * 255, cv2.COLOR_GRAY2RGB)
        mask_im = self.last_im
        cv2.circle(mask_im, cntr, 3, (255, 0, 255), 2)
        self.drawAxis(mask_im, cntr, maj_ax, (0, 255, 0), 5)
        self.drawAxis(mask_im, cntr, min_ax, (255, 0, 0), 5)
        self.pca_pub.publish(self.bridge.cv2_to_imgmsg(mask_im, "rgb8"))

        # TODO: this has a bug where the order swaps sometimes. how can we fix this?

        # rospy.loginfo(f"{vectors}")
        return np.array(
            [
                cntr + min_vec,
                cntr + min_orth_vec,
                cntr - min_vec,
                cntr - min_orth_vec,
            ]
        )

    def visual_servo_loop(
        self,
        # lambda_step=1.5,
        lambda_step=0.01,
        alpha=0.1,
        rate=5,
        use_depth=True,
        max_it=np.inf,
    ):
        """UVS Controller Loop

        Args:
            lambda_step (float, optional): step gain for positional control. Defaults to 1.0.
            alpha (float, optional): step size for broyden update. Defaults to 0.1.
            use_depth (bool, optional): whether or not to use rgb-d depth data. Defaults to True.
            max_it (int, optional): maximum allowed iterations. Defaults to infinity.

        Returns:
            tuple: total error, positional error, velocity error, velocity data, and image poses
        """
        if use_depth:
            while self.last_depth_array is None:
                rospy.sleep(0.1)

        # while self.last_target is None:
        #     rospy.sleep(0.1)

        while self.last_mask is None:
            rospy.sleep(0.1)

        rospy.loginfo("got mask")

        # iterate:
        RATE = rate
        LAMBDA = lambda_step
        ALPHA = alpha
        r = rospy.Rate(RATE)
        it = 0

        error_pos = []
        targets = []

        # pose = np.array([320, 240])
        offset = 5
        # pose = np.array([
        #     [320 - offset, 240 - offset],
        #     [320 + offset, 240 - offset],
        #     [320 - offset, 240 + offset],
        #     [320 + offset, 240 + offset],
        # ])
        pose = np.array(
            [
                [320 + offset, 240],
                [320, 240 + offset],
                [320 - offset, 240],
                [320, 240 - offset],
            ]
        )
        self.pose = pose
        jacobian, inv_jacobian = self.init_jacobian(self.pose.flatten().shape[0], is_joint_vel=True, n_joints=7)

        target_points = self.mask2pcaconstraints(self.last_mask, 1.5)
        error_p = pose - target_points
        vels = -LAMBDA * inv_jacobian @ error_p.flatten()
        rospy.loginfo(f"\n\nError: {error_p}, \n\n velocity: {vels}")

        rospy.loginfo(f"Jacobian: {jacobian}") # \n\n Inverse: {inv_jacobian}")

        rospy.loginfo(f"\n\n{jacobian @ vels}")
        # exit()
        error_p = [9999]
        while it < max_it and np.linalg.norm(error_p) > 5:
            start = rospy.get_rostime().to_sec()

            target_points = self.mask2pcaconstraints(self.last_mask, 1.5)
            self.last_target = target_points
            error_p = pose - target_points
            
            # rospy.loginfo(f"pose: {pose}")
            # rospy.loginfo(f"targets: {target_points}")
            # rospy.loginfo(error_p)
            # [[x1 y1]
            #  [x2 y2]
            #  [x3 y3]
            #  [x4 y4]]
            # exit()
            
            vels = -LAMBDA * inv_jacobian @ error_p.flatten()
            # self.move_vel(vels)
            self.move_joint_vel(vels)

            rospy.loginfo(f"velocity: {vels}")

            error_pos.append(error_p[:2])
            targets.append(self.last_target)

            r.sleep()

            target_points = self.mask2pcaconstraints(self.last_mask, 1.5)
            features_vel = (target_points - self.last_target) / (1/RATE)
            jacobian, inv_jacobian = self.broyden_update(jacobian, vels, features_vel.flatten(), ALPHA, 0.01)
            rospy.loginfo(f"Time: {rospy.get_rostime().to_sec() - start}, it: {it}")
            it += 1
            # break

        # done, stop moving
        self.move_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # reset tracking
        # self.track_reset_pub.publish(String(data="vspfc"))

        return error_pos, targets

    def __get_pixel_depth(self, pos):
        """Gets the most recent depth for the given position

        Args:
            pos (list): u, v image coordinates
        """
        return self.last_depth_array[int(pos[1]), int(pos[0])]


def main(args):
    rospy.loginfo("Starting vs node...")
    node = UVS()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down vs node...")
        node.move_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


if __name__ == "__main__":
    main(sys.argv)
