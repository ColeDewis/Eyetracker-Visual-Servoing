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
    MaskArray
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
        self.pose = None
        self.last_im = None
        self.last_target = None
        self.last_mask = None
        self.last_depth_array = None
        self.last_eye_time = rospy.Time(0)
        self.last_joints = np.zeros(7)

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
            "/sam2/masks",
            MaskArray,
            self.mask_callback
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
                im = cv2.circle(im, (int(pt[0]), int(pt[1])), 5, (0, 0, (i+1) * 60), -1)

        if self.last_target is not None:
            for i, pt in enumerate(self.last_target):
                im = cv2.circle(im, (int(pt[0]), int(pt[1])), 5, ((i+1) * 60, 0, 0), -1)
        
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

    def drawAxis(self, img, p_, q_, colour, scale):
        #https://docs.opencv.org/4.x/d1/dee/tutorial_introduction_to_pca.html
        p = list(p_)
        q = list(q_)
        
        angle = np.arctan2(p[1] - q[1], p[0] - q[0]) # angle in radians
        hypotenuse = np.sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
    
        # Here we lengthen the arrow by a factor of scale
        q[0] = p[0] - scale * hypotenuse * np.cos(angle)
        q[1] = p[1] - scale * hypotenuse * np.sin(angle)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
    
        # create the arrow hooks
        p[0] = q[0] + 9 * np.cos(angle + np.pi / 4)
        p[1] = q[1] + 9 * np.sin(angle + np.pi / 4)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
    
        p[0] = q[0] + 9 * np.cos(angle - np.pi / 4)
        p[1] = q[1] + 9 * np.sin(angle - np.pi / 4)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)

    def mask2pcaconstraints(self, mask, expand_scale: float, scale_maj: float = 0.02, scale_min: float = 0.02) -> np.ndarray:
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

        min_orth_ax = (
            cntr[0] + scale_min * eigenvectors[1, 0] * eigenvalues[1, 0],
            cntr[1] + scale_min * eigenvectors[1, 1] * eigenvalues[1, 0],
        )
        angle = np.arctan2(eigenvectors[0, 1], eigenvectors[0, 0])  # radians
        # angle is to maj ax, "Up" negative "Down" positive

        cntr = np.array(cntr)
        min_ax = np.array(min_ax)
        min_vec = np.abs(expand_scale * (min_ax - cntr))
        min_orth_vec = np.array([-min_vec[1], min_vec[0]])
        # min_orth_ax = np.array([-min_ax[1], min_ax[0]])
        # min_orth_vec = expand_scale * (min_orth_ax - cntr)
        # rospy.loginfo(f"{min_ax}, {min_orth_ax}, {maj_ax}")
        # rospy.loginfo(f"{min_vec}, {min_orth_vec}")

        # NOTE: bug has been found: i can't rotate the min_vec since its not actually a vector in the same sense. need to look at this more.
        # NOTE: honestly im pretty sure it was actually fine i dont really know what i was cooking

        # debug visualization for PCA
        # mask_im = cv2.cvtColor(mask * 255, cv2.COLOR_GRAY2RGB)
        mask_im = self.last_im
        cv2.circle(mask_im, cntr, 3, (255, 0, 255), 2)
        self.drawAxis(mask_im, cntr, maj_ax, (0, 255, 0), 5)
        self.drawAxis(mask_im, cntr, min_ax, (255, 0, 0), 5)
        self.pca_pub.publish(self.bridge.cv2_to_imgmsg(mask_im, "rgb8"))

        # TODO: this has a bug where the order swaps sometimes.
        # TODO: need to rethink these to be more stable. servoing however is working pretty nicely. the points don't seem to properly rotate with
        # the object though. Maybe just debug these for a while.

        # TODO: problem: pretty sure these like, rotate around with the object. which means they're not actually correctly tracked.
        vectors = [
            min_orth_vec - min_vec,
            min_orth_vec + min_vec,
            min_orth_vec - min_vec,
            min_orth_vec + min_vec,
        ]
        rospy.loginfo(f"{vectors}")
        return np.array(
            [
                cntr - min_orth_vec - min_vec,
                cntr - min_orth_vec + min_vec,
                cntr + min_orth_vec - min_vec,
                cntr + min_orth_vec + min_vec,
            ]
        )

    def visual_servo_loop(
        self,
        # lambda_step=1.5,
        lambda_step=0.1,
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

        # while self.last_target is None:
        #     rospy.sleep(0.1)

        while self.last_mask is None:
            rospy.sleep(0.1)
        
        rospy.loginfo("got mask")

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

        # pose = np.array([320, 240])
        offset = 5
        # NOTE: when i intentionally get the order of the middle two wrong, it is very stable, but doesn't rotate to converge properly.
        #       when i have it correct, it just sucks. i think its trying to rotate to align and doing this in combination with translation
        #       I honestly wonder if i'm just doing something wrong?
        pose = np.array([
            [320 - offset, 240 - offset],
            [320 + offset, 240 - offset],
            [320 - offset, 240 + offset],
            [320 + offset, 240 + offset],
        ])
        self.pose = pose
        error_p = [99999]

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
            # interaction for the end effector
            depth = self.__get_pixel_depth(pose[0]) if use_depth else 0.5
            pose_homog = np.column_stack([pose, np.ones(4) * depth])
            targets_homog = np.column_stack([target_points, np.ones(4) * depth])

            # error_p = np.hstack([pose - self.last_target, pose - self.last_target])
            L_bar = self.generate_interaction_matrix(
                pose_homog
            )

            L_star = self.generate_interaction_matrix(
                targets_homog
            )

            # TODO doesnt matter but should fix indices or remove since alpha/betas sum to 1 for me
            error_p[:2] = (ALPHA_1 + ALPHA_2) * error_p[:2]
            error_p[2:4] = (BETA_1 + BETA_2) * error_p[2:4]

            L = np.zeros((L_bar.shape[0] * 2, L_bar.shape[1]))
            half = L_bar.shape[0]
            L[:half, :] = ALPHA_1 * L_bar + ALPHA_2 * L_star
            L[half:, :] = BETA_1 * L_bar + BETA_2 * L_star
            L_inv = pinv(L)

            # rospy.loginfo(f"Error: {error_p}")
            error_p = np.hstack([error_p.flatten(), error_p.flatten()])
            vels_p = -L_inv @ error_p # LAMBDA
            vels_p[:3] *= 0.0 # 0.5
            vels_p[3:] *= 0.1
            # rospy.loginfo(f"Vels: {vels_p}")
            # exit()
            # if abs(rospy.get_rostime().to_sec() - self.last_eye_time.to_sec()) < 0.1:
            #     self.move_vel(vels_p)
            # else:
            #     self.move_vel(np.zeros(6))
            self.move_vel(vels_p)

            error_pos.append(error_p[:2])
            targets.append(self.last_target)

            r.sleep()
            # rospy.loginfo(f"Time: {rospy.get_rostime().to_sec() - start}, it: {it}")
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
