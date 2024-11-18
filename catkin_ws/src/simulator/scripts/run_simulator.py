import tf2_ros
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
import matplotlib.cm as cm

import numpy as np
import robosuite as suite
import robosuite.macros as macros

from robosuite.robots.single_arm import SingleArm
from robosuite.utils import camera_utils
from robosuite.utils.placement_samplers import UniformRandomSampler
from spatialmath import SE3, SO3, UnitQuaternion

from simulator.kinova_gen3_camera import KinovaGen3Camera

from kortex_driver.srv import (
    SendJointSpeedsCommand,
    SendJointSpeedsCommandRequest,
    SendJointSpeedsCommandResponse,
    SendTwistCommand,
    SendTwistCommandRequest,
    SendTwistCommandResponse,
    SendGripperCommand,
    SendGripperCommandRequest,
    SendGripperCommandResponse,
    ExecuteAction,
    ExecuteActionRequest,
    ExecuteActionResponse,
)
from kortex_driver.msg import (
    ConstrainedJointAngles,
    TwistCommand,
    ConstrainedPose,
    ActionType,
)
from custom_msgs.msg import IDPoint, Contour


class Simulator:
    # CONTROL METHODS:
    # - Joint velocity
    # - Cartesian velocity (via robot jacobian)
    # - Joint position (via direct setting)
    CONTROL_CARTESIAN_VEL = "ccv"
    CONTROL_JOINT_POS = "cjp"
    CONTROL_JOINT_VEL = "cjv"
    IDLE = "idle"

    def __init__(self):
        """Initialize the simulator."""
        rospy.init_node("simulator", disable_signals=True)
        macros.IMAGE_CONVENTION = "opencv"

        # Declare Parameters
        sim_control_type = rospy.get_param(
            "~control_type", "joint_velocity"
        )  # TODO: unused for now
        cameras = rospy.get_param("~cameras", ["birdview", "robot0_eye_in_hand"])
        camera_remappings = rospy.get_param("~camera_remappings", ["cam0", "cam2"])
        self.track_ids = rospy.get_param("~track_ids", [])
        self.edge_track_ids = rospy.get_param("~edge_track_ids", [])

        if len(cameras) != len(camera_remappings):
            rospy.loginfo(
                "Camera remapping lengths doesn't match number of cameras; ignoring."
            )
            camera_remappings = cameras

        self.camera_mapping = dict(zip(cameras, camera_remappings))

        # --- Cameras + Publishers ---
        self.cameras = cameras
        self.camera_publishers = {}
        self.depth_publishers = {}
        self.segmentation_publishers = {}
        self.segmentation_raw_publishers = {}
        self.mock_tracking_publishers = {}
        self.track_point_publishers = {}
        self.contour_publishers = {}
        self.mock_contour_image_publishers = {}

        for camera in cameras:
            self.camera_publishers[camera] = rospy.Publisher(
                f"/cameras/{self.camera_mapping[camera]}", Image, queue_size=10
            )
            self.depth_publishers[camera] = rospy.Publisher(
                f"/cameras/{self.camera_mapping[camera]}/aligned_depth",
                Image,
                queue_size=10,
            )
            self.segmentation_publishers[camera] = rospy.Publisher(
                f"/cameras/{self.camera_mapping[camera]}/segmentation",
                Image,
                queue_size=10,
            )
            self.segmentation_raw_publishers[camera] = rospy.Publisher(
                f"/cameras/{self.camera_mapping[camera]}/seg_raw", Image, queue_size=10
            )
            self.mock_tracking_publishers[camera] = rospy.Publisher(
                f"/cameras/{self.camera_mapping[camera]}/tracked_points",
                Image,
                queue_size=10,
            )
            self.mock_contour_image_publishers[camera] = rospy.Publisher(
                f"/cameras/{self.camera_mapping[camera]}/contours", Image, queue_size=10
            )
            self.track_point_publishers[camera] = rospy.Publisher(
                f"/tracking_node/{self.camera_mapping[camera]}/tracking",
                IDPoint,
                queue_size=10,
            )
            self.contour_publishers[camera] = rospy.Publisher(
                f"/tracking_node/{self.camera_mapping[camera]}/contours",
                Contour,
                queue_size=10,
            )

        self.bridge = cv_bridge.CvBridge()

        self.joint_pub = rospy.Publisher(
            "/my_gen3/joint_states", JointState, queue_size=10
        )

        # --- TF2 Publisher ----
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # http://robosuite.ai/docs/simulation/environment.html i think is where the kwargs all come from
        # cameras in "birdview" "frontview" "agentview" "sideview" "robot0_robotview" "robot0_eye_in_hand"
        config = suite.load_controller_config(default_controller="JOINT_VELOCITY")
        config["kp"] = 3  # 15
        config["velocity_limits"] = [-2, 2]
        self.seg_lvl = "element"  # element, class, instance
        rospy.loginfo(f"Controller Config: {config}")
        env = suite.make(
            env_name="Lift",  # NutAssemblyRound, Wipe (no gripper), see https://github.com/ARISE-Initiative/robosuite/tree/master/robosuite/environments/manipulation
            robots="Kinova3",
            has_renderer=True,
            has_offscreen_renderer=True,
            use_camera_obs=True,
            use_object_obs=False,
            horizon=np.inf,
            controller_configs=config,
            camera_names=self.cameras,
            camera_widths=[640] * len(self.cameras),
            camera_heights=[480] * len(self.cameras),
            camera_depths=True,
            camera_segmentations=self.seg_lvl,  # element, class, instance
            initialization_noise=None,
            # below should be there if we don't want a cube
            # placement_initializer=UniformRandomSampler(
            #     name="ObjectSampler",
            #     mujoco_objects=None,
            #     x_range=[0.5, 0.5],
            #     y_range=[0.5, 0.5],
            #     rotation=None,
            #     ensure_object_boundary_in_range=False,
            #     ensure_valid_placement=False,
            #     reference_pos=(0, 0, 0),
            #     z_offset=0.01,
            # ),
        )

        # go to /usr/local/lib/python3.8/dist-packages/robosuite/environments/manipulation/lift.py to change cube position manually

        env.reset()
        # TODO: allow camera positions to be set via launch file? or other.
        cam_mover = camera_utils.CameraMover(env=env, camera="birdview")
        pos, quat = cam_mover.get_camera_pose()
        quat = UnitQuaternion().Eul(0, np.pi / 16, np.pi / 2).vec_xyzs
        cam_mover.set_camera_pose([0.2, pos[1], 2], quat)  # x 0.1 if above uncommented

        self.env = env

        # robot model + state
        self.robot_kin = KinovaGen3Camera()
        rospy.loginfo("Robot Link Hierarchy: ")
        self.robot_kin.hierarchy()
        self.sim_robot: SingleArm = env.robots[0]
        self.control_mode = Simulator.IDLE
        self.joint_vel_target = np.zeros(7)
        self.joint_target = np.zeros(7)
        self.cartesian_vel_target = np.zeros(6)
        self.gripper_target = 0

        # --- Mock Gen3 Simulation Services ---
        self.twist_srv = rospy.Service(
            "/gen3/sim/send_twist_command", SendTwistCommand, self.twist_req_callback
        )
        self.joint_vel_srv = rospy.Service(
            "/gen3/sim/send_joint_speeds_command",
            SendJointSpeedsCommand,
            self.joint_vel_req_callback,
        )
        self.gripper_srv = rospy.Service(
            "/gen3/sim/send_gripper_command",
            SendGripperCommand,
            self.gripper_move_callback,
        )
        self.execute_action_srv = rospy.Service(
            "/gen3/sim/execute_action", ExecuteAction, self.execute_action_cb
        )

        # --- Mock Gen3 Subscribers ---
        self.cartesian_vel_sub = rospy.Subscriber(
            "/gen3/in/cartesian_velocity", TwistCommand, self.twist_topic_callback
        )

        self.run = True
        self.run_sim(env)

    def __remap_depths(self, depth_image):
        """Remaps the 0-1 depths to resemble the realsense data

        Args:
            depth_image (list): depth image
        Returns:
            list: remapped depth image
        """
        # realsense depths are in mm in uint16 format, so convert to real depths, then to that format
        depths = camera_utils.get_real_depth_map(self.env.sim, depth_image)
        depths = np.multiply(depths, 1000).astype(np.uint16)
        return depths

    def __colorize_segmentation(self, segmentation_image):
        """Remaps the integer ids from segmentation into colors

        Args:
            segmentation_image (list): segmentation image
        Returns:
            list: colorized segmentation image
        """
        # source: https://github.com/ARISE-Initiative/robosuite/blob/master/robosuite/demos/demo_segmentation.py
        segmentation_image = segmentation_image.squeeze(-1)[::-1]
        rstate = np.random.RandomState(seed=8)
        inds = np.arange(256)
        rstate.shuffle(inds)

        # TODO: this doesn't really give enough different colors for certain things
        im = (255.0 * cm.rainbow(inds[segmentation_image], 3)).astype(np.uint8)[..., :3]
        return cv2.flip(im, 0)

    def __publish_frames(self):
        """Publishes tranformation frames for joints and cameras"""
        for camera_name in self.cameras:
            extrinsics = camera_utils.get_camera_extrinsic_matrix(
                self.env.sim, camera_name
            )
            t = extrinsics[:3, -1]
            # robosuite sometimes gives invalid rotations for some reason so we don't validate them
            q = UnitQuaternion(SO3(extrinsics[:3, :3], check=False)).vec_xyzs
            tf = TransformStamped()
            tf.header.stamp = rospy.Time.now()
            tf.header.frame_id = "world"
            tf.child_frame_id = self.camera_mapping[camera_name]
            tf.transform = Transform(
                translation=Vector3(x=t[0], y=t[1], z=t[2]),
                rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
            )
            self.tf_broadcaster.sendTransform(tf)

        # publish robot base
        t = self.sim_robot.base_pos
        q = self.sim_robot.base_ori
        tf = TransformStamped()
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = "world"
        tf.child_frame_id = "robot_origin"
        tf.transform = Transform(
            translation=Vector3(x=t[0], y=t[1], z=t[2]),
            rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
        )
        self.tf_broadcaster.sendTransform(tf)

        # publish robot link frames.
        # NOTE this doesn't publish gripper frames, TODO add?, would have to adapt robotics toolbox
        frames = [
            "base_link",
            "shoulder_link",
            "half_arm_1_link",
            "half_arm_2_link",
            "forearm_link",
            "spherical_wrist_1_link",
            "spherical_wrist_2_link",
            "bracelet_link",
            "end_effector_link",
            "camera_link",
            "camera_depth_frame",
            "camera_color_frame",
            "tool_frame",
        ]
        forward_kin = self.robot_kin.fkine_all(self.sim_robot._joint_positions)
        forward_kin.pop()  # duplicate at the end
        kin: SE3
        for i, kin in enumerate(forward_kin):
            t = kin.t
            q = UnitQuaternion(kin.R).vec_xyzs
            tf = TransformStamped()
            tf.header.stamp = rospy.Time.now()
            tf.header.frame_id = "robot_origin"
            tf.child_frame_id = frames[i]
            tf.transform = Transform(
                translation=Vector3(x=t[0], y=t[1], z=t[2]),
                rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
            )
            self.tf_broadcaster.sendTransform(tf)

    def __publish_joint_states(self):
        """Publishes the robot's Joint State.

        Unlike the kinova itself, this does not have detailed gripper info - only the "finger joint"
        """
        js = JointState()
        js.header.stamp = rospy.Time().now()
        js.name = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "joint_7",
        ]
        js.position = self.sim_robot._joint_positions
        js.velocity = self.sim_robot._joint_velocities
        js.effort = self.sim_robot.torques
        self.joint_pub.publish(js)

    def __publish_cameras(self, obs):
        """Publishes Cameras from Simulation

        Args:
            obs (dict): observations returned from simulation step
        """
        for camera in self.cameras:
            image = obs[f"{camera}_image"]

            # depth
            depth_img = obs[f"{camera}_depth"]
            depth_img = self.__remap_depths(depth_img)

            # colorized segmentation
            seg_img = obs[f"{camera}_segmentation_{self.seg_lvl}"]
            seg_img = self.__colorize_segmentation(seg_img)

            # raw segmentation classes
            seg_raw = obs[f"{camera}_segmentation_{self.seg_lvl}"]
            # rospy.loginfo(f"{np.unique(seg_raw)}")

            self.__mock_tracking(camera, np.copy(image), seg_raw)
            self.__mock_edge_tracking(camera, np.copy(image), seg_raw)

            image = self.bridge.cv2_to_imgmsg(image, "rgb8")
            depth_img = self.bridge.cv2_to_imgmsg(depth_img, "16UC1")
            seg_raw = self.bridge.cv2_to_imgmsg(seg_raw, "passthrough")
            seg_img = self.bridge.cv2_to_imgmsg(seg_img, "rgb8")
            self.camera_publishers[camera].publish(image)
            self.depth_publishers[camera].publish(depth_img)
            self.segmentation_publishers[camera].publish(seg_img)
            self.segmentation_raw_publishers[camera].publish(seg_raw)

    def __mock_tracking(self, camera, base_image, segmentation_image):
        """Runs mock tracking on a segmentation image. Tracks any IDs that were given in the launch arg.

        Args:
            camera (str): name of camera being used
            base_image (list): image to draw the tracking on
            segmentation_image (list): segmentation rendered image
        """
        for id in self.track_ids:
            im_bw = cv2.inRange(segmentation_image, id, id)

            contours, hierarchy = cv2.findContours(
                im_bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue

            # centroid
            cX = M["m10"] / M["m00"]
            cY = M["m01"] / M["m00"]

            idpt = IDPoint(id="sim", x=cX, y=cY)
            idpt.header.stamp = rospy.get_rostime()
            self.track_point_publishers[camera].publish(idpt)
            base_image = cv2.circle(base_image, (int(cX), int(cY)), 0, (255, 0, 0), 5)

        base_image = self.bridge.cv2_to_imgmsg(base_image, "rgb8")
        self.mock_tracking_publishers[camera].publish(base_image)

    def __mock_edge_tracking(self, camera, base_image, segmentation_image):
        """Runs mock edge tracking/contour tracking on the IDs in the launch argument

        Args:
            camera (str): name of camera being used
            base_image (list): base camera image
            segmentation_image (list): segmentation rendered image
        """
        for id in self.edge_track_ids:
            im_bw = cv2.inRange(segmentation_image, id, id)  # id, id

            contours, hierarchy = cv2.findContours(
                im_bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue

            contour = max(contours, key=cv2.contourArea)

            contour = np.array(contour)
            min_el = max(contour, key=lambda x: x[0][1])
            min_idx = np.where(np.all(contour == min_el, axis=1))[0][0]
            contour = np.row_stack([contour[min_idx:], contour[:min_idx]])
            contour = np.unique(contour, axis=1)

            c = Contour(
                id=str(id), x_points=contour[:, 0, 0], y_points=contour[:, 0, 1]
            )
            c.header.stamp = rospy.get_rostime()
            self.contour_publishers[camera].publish(c)
            base_image = cv2.drawContours(base_image, contours, 0, (255, 0, 0), 3)

        base_image = self.bridge.cv2_to_imgmsg(base_image, "rgb8")
        self.mock_contour_image_publishers[camera].publish(base_image)

    def twist_req_callback(self, req: SendTwistCommandRequest):
        """Handler for a twist command request

        Args:
            req (SendTwistCommandRequest): twist command request message
        """
        req.input.twist.angular_x = np.deg2rad(req.input.twist.angular_x)
        req.input.twist.angular_y = np.deg2rad(req.input.twist.angular_y)
        req.input.twist.angular_z = np.deg2rad(req.input.twist.angular_z)
        self.twist_topic_callback(req.input)
        return SendTwistCommandResponse()

    def twist_topic_callback(self, msg: TwistCommand):
        """Handler for twist (cartesian velocity) topic

        Args:
            msg (TwistCommand): twist command message
        """
        twist = np.array(
            [
                msg.twist.linear_x,
                msg.twist.linear_y,
                msg.twist.linear_z,
                msg.twist.angular_x,
                msg.twist.angular_y,
                msg.twist.angular_z,
            ]
        )
        self.control_mode = Simulator.CONTROL_CARTESIAN_VEL
        self.cartesian_vel_target = twist

    def joint_vel_req_callback(self, req: SendJointSpeedsCommandRequest):
        """Handles joint velocity requests

        Args:
            req (SendJointSpeedsCommandRequest): joint speed request message
        """
        self.control_mode = Simulator.CONTROL_JOINT_VEL
        self.joint_vel_target = np.deg2rad([x.value for x in req.input.joint_speeds])
        rospy.loginfo(f"{self.joint_vel_target}")
        return SendJointSpeedsCommandResponse()

    def gripper_move_callback(self, req: SendGripperCommandRequest):
        """Handles gripper movement requests and moves the gripper

        Args:
            req (SendGripperCommandRequest): gripper command request.
                Only used field is finger.value, which should be from -1 to 1, and is a velocity, NOT a position.
        """
        self.gripper_target = req.input.gripper.finger[0].value
        return SendGripperCommandResponse()

    def execute_action_cb(self, req: ExecuteActionRequest):
        """General callback for execute action to manage various kinova behaviors

        Args:
            req (ExecuteActionRequest): action request message
        """
        if req.input.handle.identifier == 1001:
            return self.pose_move_callback(req)
        else:
            return self.joint_move_callback(req)

    def joint_move_callback(self, req: ExecuteActionRequest):
        """Handler for joint move request.

        Uses a higher level ExecuteAction service; if other methods are needed in future this will need refactoring

        Args:
            req (ExecuteActionRequest): execute action request message, assumed to contain a "reach_joint_angles" entry filled
        """
        constrained_angles: ConstrainedJointAngles = (
            req.input.oneof_action_parameters.reach_joint_angles[0]
        )
        angles = [x.value for x in constrained_angles.joint_angles.joint_angles]
        self.control_mode = Simulator.CONTROL_JOINT_POS
        self.joint_target = np.deg2rad(angles)
        return ExecuteActionResponse()

    def pose_move_callback(self, req: ExecuteActionRequest):
        """Handler for pose move request. Uses robotics toolbox inverse kinematics.

        Args:
            req (ExecuteActionRequest): execute action message, assumed to contain a "reach pose" entry filled
        """
        pose: ConstrainedPose = req.input.oneof_action_parameters.reach_pose[0]

        # TODO: figure out the weird upside down issue if we use orientation
        current: SE3 = self.robot_kin.fkine(self.sim_robot._joint_positions)
        target = SE3().Rt(
            current.R,
            [pose.target_pose.x, pose.target_pose.y, pose.target_pose.z],
        )
        sol = self.robot_kin.ikine_LM(
            target, q0=self.sim_robot._joint_positions, end="camera_color_frame"
        )

        rospy.loginfo(f"success: {sol.success}, q: {sol.q}")
        if sol.success or True:
            self.joint_target = sol.q
            self.control_mode = Simulator.CONTROL_JOINT_POS
        return ExecuteActionResponse()

    def run_sim(self, env):
        """Run the given environment as the simulator

        Args:
            env (Environment): Robosuite env
        """
        while self.run:
            self.__publish_frames()
            self.__publish_joint_states()

            step = np.zeros(8)
            if self.control_mode == Simulator.CONTROL_CARTESIAN_VEL:
                j_0 = self.robot_kin.jacob0(
                    self.sim_robot._joint_positions, end="camera_color_frame"
                )
                q_dot = np.linalg.pinv(j_0) @ self.cartesian_vel_target
                # rospy.loginfo(f"q_dot: {q_dot}, vels: {self.cartesian_vel_target}")
                step = np.hstack([q_dot, 0])
            elif self.control_mode == Simulator.CONTROL_JOINT_VEL:
                step = np.hstack([self.joint_vel_target, 0])
            elif self.control_mode == Simulator.CONTROL_JOINT_POS:
                self.sim_robot.set_robot_joint_positions(self.joint_target)

            self.sim_robot.grip_action(self.sim_robot.gripper, [self.gripper_target])
            obs, _, _, _ = env.step(step)

            self.__publish_cameras(obs)

            env.render()  # render on display

        env.close()


def main():
    rospy.loginfo("Starting the simulator ...")
    node = Simulator()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down simulator...")


if __name__ == "__main__":
    main()
