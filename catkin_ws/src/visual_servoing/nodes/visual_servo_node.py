import sys
import rospy
import cv2
import tf2_ros
import tf_conversions

import numpy as np
import actionlib

from kortex_bringup import KinovaGen3

from std_msgs.msg import Empty
from visual_servoing.msg import Error, TrackedPoints
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Transform

from custom_msgs.msg import (
    UVSRequestAction,
    UVSRequestGoal,
    UVSRequestResult,
    UVSRequestFeedback,
    UVSWaypointFollowAction,
    UVSWaypointFollowGoal,
    UVSWaypointFollowResult,
    UVSWaypointFollowFeedback,
    IBVSRequestAction,
    IBVSRequestGoal,
    IBVSRequestResult,
    IBVSRequestFeedback,
    IBVSWaypointFollowAction,
    IBVSWaypointFollowGoal,
    IBVSWaypointFollowResult,
    IBVSWaypointFollowFeedback,
    IBVSWaypoint,
    DesiredFeature,
    Point2D,
    Matrix,
)
from custom_msgs.srv import SwapWaypoint, SwapWaypointRequest


class VisualServoing:
    """Visual Servoing Node."""

    def __init__(self):
        """Initialize visual servoing, setting up the required subscribers."""

        rospy.init_node("visual_servo")
        rospy.loginfo("Starting VisualServoing Node")

        self.kinova: KinovaGen3 = KinovaGen3()

        # state variables
        self.latest_eef = None
        self.latest_error = None
        self.latest_joints = None
        self.jacobian = None
        self.inverse_jacobian = None
        self.running = False
        self.L = None

        # holds last state for UVS
        self.prev_eef = None
        self.prev_joints = None
        self.prev_error = None

        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # TODO: the servoing node shouldn't enforce any particular starting position, we expect those who use it to do that
        # Kinova has 7 joints.
        # For fine VS, we may want to use all 7; however generally we only want to use 4. Those are:
        #   - idx 0: rotation of the arm base
        #   - idx 1: rotation of 2nd arm link
        #   - idx 3: rotation of 3rd arm link
        #   - idx 5: rotation of 4th arm link
        # Anything more than this shouldn't be necessary for large, coarse motions
        self.joints = np.deg2rad(
            np.array(
                [
                    -0.1336059570312672,
                    -28.57940673828129,
                    -179.4915313720703,
                    -147.7,
                    0.06742369383573531,
                    -57.420898437500036,
                    89.88030242919922,
                ]
            )
        )

        # NOTE: temp starting angles.
        # sideways digs
        # self.joints = np.array([1.1425696108900794, 0.5377173243802722, -2.942464636775151, -1.1228542293402226, -0.057825216748890185, -1.4166024954726169, 1.2148203723156021])
        # self.joints = np.array([1.3961009487689688, 0.39095677513456706, -2.9525212656835085, -0.8713112075311864, -0.11094196469051987, -1.7232276781077616, 1.661490011401436])
        # self.joints = np.array([1.3961492851427537, 0.5085698917767092, -2.9526480321513966, -1.1901049058191218, -0.10090184738092134, -1.2903878345627096, 1.6620046672820936])

        # tabletop manip
        # self.joints = np.array([-0.19655300635664474, 0.8299573758083271, -2.8110765191316562, -1.0309490727538062, -0.31231795578129784, -1.1241426666756027, 1.7763640002834045])

        # jackal side dig
        self.joints = np.array(
            [
                1.288626419817731,
                0.7594544470062233,
                -3.040972302336092,
                -1.3282878126678455,
                -0.10521723361128643,
                -0.8572284116868474,
                1.3454223248053059,
            ]
        )

        self.kinova.send_joint_angles(np.copy(self.joints))

        # --- Actions ---
        self.uvs_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
            "/visual_servo_node/uvs_action",
            UVSRequestAction,
            execute_cb=self.uvs_action_callback,
            auto_start=False,
        )
        self.uvs_server.start()
        self.ibvs_server: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
            "/visual_servo_node/ibvs_action",
            IBVSRequestAction,
            execute_cb=self.ibvs_action_callback,
            auto_start=False,
        )
        self.ibvs_server.start()
        self.uvs_waypoint_server: actionlib.SimpleActionServer = (
            actionlib.SimpleActionServer(
                "/visual_servo_node/uvs_waypoint_follow",
                UVSWaypointFollowAction,
                execute_cb=self.uvs_waypoint_follow_action,
                auto_start=False,
            )
        )
        self.uvs_waypoint_server.start()
        self.ibvs_waypoint_server: actionlib.SimpleActionServer = (
            actionlib.SimpleActionServer(
                "/visual_servo_node/ibvs_waypoint_follow",
                IBVSWaypointFollowAction,
                execute_cb=self.ibvs_waypoint_follow_action,
                auto_start=False,
            )
        )
        self.ibvs_waypoint_server.start()

        # --- Subscribers ---
        self.error_subscriber = rospy.Subscriber(
            "/image_error", Error, self.img_error_callback
        )
        self.eef_subscriber = rospy.Subscriber(
            "/eef_pos", TrackedPoints, self.eef_pose_callback
        )

        # --- Service Proxies ---
        rospy.wait_for_service("/tracking_node/swap_waypoint")
        self.swap_waypoint_service = rospy.ServiceProxy(
            "/tracking_node/swap_waypoint", SwapWaypoint
        )

        rospy.loginfo("Waiting for messages on /image_error and /eef_pos")
        rospy.wait_for_message("/image_error", Error)
        rospy.wait_for_message("/eef_pos", TrackedPoints)

        rospy.loginfo("Visual Servoing is ready to run!")

    def eef_pose_callback(self, data: TrackedPoints):
        """Callback for getting the latest eef pose

        Args:
            data (TrackedPoints): last eef pose
        """
        if len(data.points) == 0:
            return
        self.latest_eef = np.array([data.points[0].x, data.points[0].y])

    def img_error_callback(self, data: Error):
        """Callback for getting the latest image error

        Args:
            data (Error): last error
        """
        if len(data.error) == 0:
            return
        self.latest_error = np.asarray([data.error])

    def move_pos(self, delta, num_joints):
        """Move the kinova arm by delta

        Args:
            delta (list): delta for the num_joints
            num_joints (int): 4 or 7 for degrees of freedom
        """

        current_joints = rospy.wait_for_message("/my_gen3/joint_states", JointState)
        current_joints = list(current_joints.position[:7])
        self.joints = current_joints

        if num_joints == 4:
            self.joints[0] += delta[0]
            self.joints[1] += delta[1]
            self.joints[3] += delta[2]
            self.joints[5] += delta[3]
        else:
            for i in range(7):
                self.joints[i] += delta[i]

        # NOTE: motion blocks!
        rospy.loginfo(f"Sending Joints: {self.joints}")
        self.kinova.send_joint_angles(np.copy(self.joints))
        rospy.loginfo("Joint Move Complete.")

        if num_joints == 4:
            self.latest_joints = np.array(
                [self.joints[0], self.joints[1], self.joints[3], self.joints[5]]
            )
        else:
            self.latest_joints = np.copy(self.joints)

    def move_vel(self, velocities):
        """Send the given twist to the kinova

        Args:
            velocities (list): list of twist, 3 linear in m/s, 3 angular in deg/s
        """
        if np.linalg.norm(velocities) == 0:
            self.kinova.send_twist(vels=velocities, duration_ms=0)
        else:
            normalized_vels = velocities / np.linalg.norm(velocities)
            normalized_vels = normalized_vels / 25  # TODO: evaluate
            rospy.loginfo(f"Normalized Velocities: {normalized_vels}")

            self.kinova.send_twist(vels=normalized_vels, duration_ms=0)

    def get_adjoint_matrix(self, rot, trans):
        """Given a rotation matrix and translation vector, returns the adjoint representation of the transformation in R3

        Args:
            rot (list): 3x3 rotation matrix from camera to eef
            trans (list): vector in R3 representing the translation from camera to eef

        Returns:
            matrix: 6x6 numpy matrix representing the adjoint matrix
        """

        # rot 3x3 rot matrix, trans is vector in R3
        skew_t = np.matrix(
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

    def broyden_update(self, learn_rate: float):
        """Update Jacobian with a Broyden Update.

        Args:
            learn_rate (float): learn rate for the broyden update
        """
        current_eef = self.latest_eef
        delta_eef = self.prev_eef - current_eef

        # if tracked change is too small, return
        if np.linalg.norm(delta_eef) < 10:
            return

        delta_x = self.latest_joints - self.prev_joints
        delta_e = (self.latest_error - self.prev_error)[0]

        # prevent divide by what is essentially 0 in the case of jitters in trackers, but the arm hasn't moved.
        if np.linalg.norm(delta_x) < 0.0001:
            return

        # broyden update
        numerator = np.subtract(delta_e.T, self.jacobian @ delta_x.T)
        update = (
            np.reshape(numerator, (numerator.size, 1))
            @ np.reshape(delta_x, (1, delta_x.size))
        ) / (delta_x @ delta_x.T)

        self.jacobian = self.jacobian + learn_rate * update
        self.inverse_jacobian = np.linalg.pinv(self.jacobian)

        self.prev_error = self.latest_error
        self.prev_eef = self.latest_eef
        self.prev_joints = self.latest_joints

    def secant_jacobian(self, num_joints: int):
        """Generate a jacobian using secant constraints

        Args:
            num_joints (int): number of joints, 4 or 7.
        """

        # TODO: test this method on the robot

        self.jacobian = np.zeros((self.latest_error.size, num_joints))

        rand_moves = np.random.uniform(
            low=-0.025, high=0.025, size=(num_joints, num_joints)
        )

        error_mat = np.zeros((num_joints, len(self.latest_error)))
        for i, move in enumerate(rand_moves):
            initial_error = self.latest_error
            self.move_pos(move, num_joints)
            rospy.sleep(0.1)
            after_error = self.latest_error

            error_mat[i, :] = after_error - initial_error

        J_t = np.linalg.lstsq(rand_moves, error_mat, rcond=None)

        self.jacobian = np.transpose(J_t)
        self.inverse_jacobian = np.linalg.pinv(self.jacobian)

        if num_joints == 4:
            self.prev_joints = np.array(
                [self.joints[0], self.joints[1], self.joints[3], self.joints[5]]
            )
        else:
            self.prev_joints = np.copy(self.joints)

    def update_jacobian(self, num_joints: int):
        """Generate an initial jacobian, or update to reset it by doing basis movements.

        Args:
            num_joints (int): number of joints to use. Should be 4 or 7 (for kinova)
        """
        self.jacobian = np.zeros((self.latest_error.size, num_joints))
        delta = 0.025

        # loop for each joint
        move = [0.0] * num_joints
        for i in range(num_joints):
            initial_error = self.latest_error

            # move robot
            move[i] = delta
            self.move_pos(move, num_joints)

            rospy.sleep(0.1)
            after_error = self.latest_error
            rospy.loginfo(f"ERROR BEFORE: {initial_error}")
            rospy.loginfo(f"ERROR AFTER: {after_error}")

            # move back
            move[i] = -delta
            self.move_pos(move, num_joints)

            self.jacobian[:, i] = (after_error - initial_error) / delta

            rospy.loginfo(f"JACOBIAN: {self.jacobian}")
            move = [0.0] * num_joints

        self.inverse_jacobian = np.linalg.pinv(self.jacobian)

        self.prev_error = self.latest_error
        self.prev_eef = self.latest_eef

        if num_joints == 4:
            self.prev_joints = np.array(
                [self.joints[0], self.joints[1], self.joints[3], self.joints[5]]
            )
        else:
            self.prev_joints = np.copy(self.joints)

    def get_move_pos(self, step_size: float) -> list:
        """Get the delta for the next motion.

        Args:
            step_size (float): lambda for equation for getting the delta move

        Returns:
            list: delta joint movement
        """
        return -step_size * (self.inverse_jacobian @ self.latest_error.T).T

    def generate_interaction_matrix(
        self, desired_points, desired_camera_depth, camera_idx, no_z: bool = False
    ) -> list:
        """Generates the interaction matrix for velocity control for given image points and camera transformation.

        Args:
            desired_points (list): list of x,y desired image points
            desired_camera_depth (float): desired depth (Z) for interaction matrix
            camera_idx (int): index of camera these points are for - needed to get the transform from camera to tool link.
            no_z (bool, optional): true if we should not allow z translational movement, false otherwise. Defaults to false

        Returns:
            list: interaction matrix
        """
        Z = desired_camera_depth

        # get transform from camera to base
        try:
            eef_transform: TransformStamped = self.tf_buffer.lookup_transform(
                "base_link", f"cam{camera_idx}", rospy.Time(0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn(
                f"Failed to lookup transform to tool_frame from camera_{camera_idx}: {e}"
            )
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
            x, y = point
            part_L = np.matrix(
                [
                    [-1 / Z, 0, x / Z, x * y, -(1 + x * x), y],
                    [0, -1 / Z, y / Z, 1 + y * y, -x * y, -x],
                ]
            )

            # transform interaction matrix to base. see https://inria.hal.science/inria-00350283/document
            part_L = part_L @ adj_mat
            if L is None:
                L = part_L
            else:
                L = np.row_stack([L, part_L])

        if no_z:
            L[:, 2] = 0.0

        return L

    def get_move_vel(self, interaction_mat: list, step_size: float) -> list:
        """Get the desired end effector velocities for velocity control

        Args:
            interaction_mat (list): interaction matrix
            step_size (float): lambda for equation for getting the velocity

        Returns:
            list: twist velocities
        """
        return -step_size * (np.linalg.pinv(interaction_mat) @ self.latest_error.T)

    # TODO: uvs_action_callback should be able to change to call __uvs_waypoint_servo_next
    def uvs_action_callback(self, goal: UVSRequestGoal):
        """Callback for UVS action

        Args:
            goal (UVSRequestGoal): goal message
        """
        print("???")
        learn_rate = goal.learn_rate
        init_step_size = goal.init_step_size
        max_step_size = goal.max_step_size
        max_it = goal.max_it
        threshold = goal.threshold
        num_joints = goal.num_joints
        has_init_jacobian = goal.has_init_jacobian

        feedback: UVSRequestFeedback = UVSRequestFeedback()
        result: UVSRequestResult = UVSRequestResult()

        rospy.loginfo(
            f"Starting VS Converge with max iterations {max_it} and starting step_size {init_step_size}"
        )

        if has_init_jacobian:
            data: Matrix = goal.starting_jacobian
            n = data.n
            m = data.m
            cached = np.array(data.data).reshape((n, m))
            self.jacobian = cached
            self.inverse_jacobian = np.linalg.pinv(cached)
        else:
            self.update_jacobian(num_joints)

        # initial jacobian is sent back in result
        init_j = Matrix()
        init_j.n = self.jacobian.shape[0]
        init_j.m = self.jacobian.shape[1]
        init_j.data = self.jacobian.flatten()
        result.initial_jacobian = init_j

        initial_error = np.linalg.norm(self.latest_error)
        initial_stepsize = init_step_size
        error_increased_ct = 0
        prev_error = initial_error

        for i in range(max_it):
            # check for action preemption
            if self.uvs_server.is_preempt_requested():
                rospy.loginfo(f"Visual Servoing preempted on iteration {i}")
                result.converged = False
                self.uvs_server.set_preempted(result=result)
                return

            self.broyden_update(learn_rate)

            # calculate new stepsize, and move.
            step = (initial_stepsize / (1 / initial_error)) * (
                1 / np.linalg.norm(self.latest_error)
            )
            step = max(0.0, min(step, max_step_size))
            rospy.loginfo(f"STEP: {step}")
            delta_move = self.get_move_pos(step)[0]
            rospy.loginfo(f"DELTA MOVE: {delta_move}")
            rospy.loginfo(f"LAST ERROR: {self.latest_error}")

            self.move_pos(delta_move, num_joints)

            if np.linalg.norm(self.latest_error) > np.linalg.norm(prev_error):
                error_increased_ct += 1
                if error_increased_ct > 5:
                    self.update_jacobian(num_joints)
                    error_increased_ct = 0
            else:
                error_increased_ct = 0

            prev_error = self.latest_error

            # get error norm for action feedback
            feedback.remaining_error_norm = np.linalg.norm(self.latest_error)
            feedback.iteration = i
            self.uvs_server.publish_feedback(feedback)

            if np.linalg.norm(delta_move) < 0.001:
                rospy.loginfo("Servoing stopped due to negligable delta move.")
                result.converged = False
                self.uvs_server.set_succeeded(result)
                return

            if np.linalg.norm(self.latest_error) < threshold:
                rospy.loginfo(f"Servoing converged after {i} iterations")
                result.converged = True
                self.uvs_server.set_succeeded(result)
                return

        rospy.logwarn(f"UVS did not converge after {max_it} iterations.")
        result.converged = False
        self.uvs_server.set_succeeded(result)
        return

    def uvs_waypoint_follow_action(self, goal: UVSWaypointFollowGoal):
        """Follow the currently active waypoints using UVS.

        Args:
            goal (UVSWaypointFollowGoal): goal for the action
        """
        learn_rate = goal.learn_rate
        init_step_size = goal.init_step_size
        max_step_size = goal.max_step_size
        max_it = goal.max_it
        num_joints = goal.num_joints
        threshold = goal.threshold
        waypoint_ids = goal.waypoint_ids
        rospy.loginfo(f"Starting UVS Waypoint Follow.")

        waypoint_num = 1
        while True:
            # servo to the next waypoint
            waypoint_servo_succeeded, waypoint_servo_preempted = (
                self.__uvs_waypoint_servo_next(
                    learn_rate,
                    max_it,
                    num_joints,
                    init_step_size,
                    max_step_size,
                    threshold,
                    waypoint_num,
                )
            )

            if waypoint_servo_succeeded:
                success = len(waypoint_ids) != 0
                no_more_waypoints = False
                for wpt_id in waypoint_ids:
                    resp = self.swap_waypoint_service.call(
                        SwapWaypointRequest(id=wpt_id)
                    )
                    success = success and resp.success  # all must succeed
                    no_more_waypoints = (
                        no_more_waypoints or resp.no_more_waypoints
                    )  # any having none left causes us to stop.

                waypoint_num += 1
                rospy.wait_for_message("/image_error", Error)

                if success and no_more_waypoints:
                    rospy.loginfo(f"Waypoint following completed successfully.")
                    self.uvs_waypoint_server.set_succeeded(
                        UVSWaypointFollowResult(converged=True)
                    )
                    return
                elif not success:
                    rospy.loginfo(
                        f"Waypoint following error: could not swap waypoint. Stopping."
                    )
                    self.uvs_waypoint_server.set_succeeded(
                        UVSWaypointFollowResult(converged=False)
                    )
                    return

            elif waypoint_servo_preempted:
                rospy.loginfo(f"UVS Waypoint following was preempted")
                self.uvs_waypoint_server.set_preempted(
                    UVSWaypointFollowResult(converged=False)
                )
                return
            else:
                rospy.loginfo(f"Waypoint following did not converge.")
                self.uvs_waypoint_server.set_succeeded(
                    UVSWaypointFollowResult(converged=False)
                )
                return

    def __uvs_waypoint_servo_next(
        self,
        learn_rate: float,
        max_it: int,
        num_joints,
        init_step_size: float,
        max_step_size: float,
        threshold: float,
        waypoint_num: int,
    ) -> bool:
        """Servos to the next waypoint in the waypoint action

        Args:
            learn_rate (float): learn rate for broyden updates
            max_it (int): max allowed iterations
            num_joints (int): degrees of freedom being used
            init_step_size (float): starting step size
            max_step_size (float): max allowed step size
            threshold (float): error threshold to be "converged"
            waypoint_num (int): current waypoint number

        Returns:
            bool, bool: first is true if converged, false otherwise. second is true if preempted, false otherwise
        """
        # use UVS to servo to waypoint
        initial_error = np.linalg.norm(self.latest_error)
        initial_stepsize = init_step_size
        error_increased_ct = 0
        prev_error = initial_error

        self.update_jacobian(num_joints)
        for i in range(max_it):
            # check for action preemption
            if self.uvs_waypoint_server.is_preempt_requested():
                rospy.loginfo(
                    f"UVS Waypoint Follow preempted on waypoint {waypoint_num}"
                )
                return False, True

            self.broyden_update(learn_rate)
            step = (initial_stepsize / (1 / initial_error)) * (
                1 / np.linalg.norm(self.latest_error)
            )
            step = max(0.0, min(step, max_step_size))
            delta_move = self.get_move_pos(step)[0]
            self.move_pos(delta_move, num_joints)
            rospy.loginfo(f"step - {step}, delta_move - {delta_move}")

            if np.linalg.norm(self.latest_error) > np.linalg.norm(prev_error):
                error_increased_ct += 1
                if error_increased_ct > 5:
                    self.update_jacobian(num_joints)
                    error_increased_ct = 0
            else:
                error_increased_ct = 0

            prev_error = self.latest_error

            # get error norm for action feedback
            feedback: UVSWaypointFollowFeedback = UVSWaypointFollowFeedback()
            feedback.remaining_error_norm = np.linalg.norm(self.latest_error)
            feedback.iteration = i
            feedback.waypoint_num = waypoint_num
            self.uvs_waypoint_server.publish_feedback(feedback)

            if np.linalg.norm(delta_move) < 0.001:
                rospy.loginfo(
                    f"Servoing to waypoint {waypoint_num} stopped due to negligable delta move."
                )
                return False, False

            if np.linalg.norm(self.latest_error) < threshold:
                rospy.loginfo(
                    f"Servoing to waypoint {waypoint_num} converged after {i} iterations"
                )
                return True, False

        return False, False

    # TODO: ibvs_action_callback should be able to change to call __ibvs_waypoint_servo_next
    def ibvs_action_callback(self, goal: IBVSRequestGoal):
        """Callback for the IBVS servoing action

        Args:
            goal (IBVSRequestGoal): goal for the action
        """
        step_size = goal.step_size
        max_it = goal.max_it
        threshold = goal.threshold
        desired_depth = goal.desired_depth
        desired_points = goal.desired_features

        features = {}
        feature: DesiredFeature
        for feature in desired_points:
            if features.get(feature.camera_idx, None) is not None:
                features[feature.camera_idx].append([feature.point.x, feature.point.y])
            else:
                features[feature.camera_idx] = [[feature.point.x, feature.point.y]]

        interaction_mat = []
        for key, value in features.items():
            L = self.generate_interaction_matrix(
                desired_points=value, desired_camera_depth=desired_depth, camera_idx=key
            )
            interaction_mat.append(L)
        interaction_mat = np.asarray(interaction_mat)
        rospy.loginfo(f"INTERACTION MATRIX: {interaction_mat}")

        feedback: IBVSRequestFeedback = IBVSRequestFeedback()
        result: IBVSRequestResult = IBVSRequestResult()

        rospy.loginfo(
            f"Starting IBVS with max iterations {max_it} and starting step_size {step_size}"
        )

        r = rospy.Rate(30)
        for i in range(max_it):
            # check for action preemption
            if self.ibvs_server.is_preempt_requested():
                rospy.loginfo(f"IBVS preempted on iteration {i}")
                result.converged = False
                self.ibvs_server.set_preempted(result=result)
                self.move_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # stop moving
                return

            # calculate new velocity, and move.
            vels = self.get_move_vel(
                interaction_mat=interaction_mat, step_size=step_size
            )[0]

            # get error norm for action feedback
            feedback.remaining_error_norm = np.linalg.norm(self.latest_error)
            feedback.iteration = i
            self.ibvs_server.publish_feedback(feedback)

            self.move_vel(velocities=vels)

            if np.linalg.norm(self.latest_error) < threshold:
                self.move_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # stop moving
                rospy.loginfo(f"IBVS converged after {i} iterations")
                result.converged = True
                self.ibvs_server.set_succeeded(result)
                return

            r.sleep()

        rospy.logwarn(f"IBVS did not converge after {max_it} iterations.")
        result.converged = False
        self.move_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # stop moving
        self.ibvs_server.set_succeeded(result)
        return

    def ibvs_waypoint_follow_action(self, goal: IBVSWaypointFollowGoal):
        """Follow waypoints using IBVS.

        Args:
            goal (IBVSWaypointFollowGoal): goal for the action
        """
        step_size = goal.step_size
        max_it = goal.max_it
        threshold = goal.threshold
        ibvs_waypoints = goal.ibvs_waypoints
        waypoint_ids = goal.waypoint_ids
        rospy.loginfo(f"Starting IBVS Waypoint Follow.")

        waypoint_num = 1
        while True:
            ibvs_waypoint: IBVSWaypoint = ibvs_waypoints[waypoint_num - 1]
            desired_depth = ibvs_waypoint.desired_depth
            desired_features = ibvs_waypoint.desired_features
            # servo to the next waypoint
            waypoint_servo_succeeded, waypoint_servo_preempted = (
                self.__ibvs_waypoint_servo_next(
                    step_size,
                    max_it,
                    threshold,
                    desired_depth,
                    desired_features,
                    waypoint_num,
                )
            )

            if waypoint_servo_succeeded:
                success = True
                no_more_waypoints = False
                for wpt_id in waypoint_ids:
                    resp = self.swap_waypoint_service.call(
                        SwapWaypointRequest(id=wpt_id)
                    )
                    success = success and resp.success  # all must succeed
                    no_more_waypoints = (
                        no_more_waypoints or resp.no_more_waypoints
                    )  # any having none left causes us to stop.

                waypoint_num += 1
                rospy.wait_for_message("/image_error", Error)

                if success and no_more_waypoints:
                    rospy.loginfo(f"IBVS Waypoint following completed successfully.")
                    self.ibvs_waypoint_server.set_succeeded(
                        IBVSWaypointFollowResult(converged=True)
                    )
                    return
                elif not success:
                    rospy.loginfo(
                        f"IBVS Waypoint following error: could not swap waypoint. Stopping."
                    )
                    self.ibvs_waypoint_server.set_succeeded(
                        IBVSWaypointFollowResult(converged=False)
                    )
                    return
            elif waypoint_servo_preempted:
                rospy.loginfo(f"IBVS Waypoint following was preempted")
                self.ibvs_waypoint_server.set_preempted(
                    IBVSWaypointFollowResult(converged=False)
                )
                return
            else:
                rospy.loginfo(f"IBVS Waypoint following did not converge.")
                self.ibvs_waypoint_server.set_succeeded(
                    IBVSWaypointFollowResult(converged=False)
                )
                return

    def __ibvs_waypoint_servo_next(
        self,
        step_size: float,
        max_it: int,
        threshold: float,
        desired_depth: float,
        desired_features: list,
        waypoint_num: int,
    ):
        """Servos to the next waypoint in the waypoint action

        Args:
            step_size (float): step size for motion
            max_it (int): max allowed iterations
            threshold (float): error threshold to be "converged"
            desired_depth (float): desired depth for interaction matrix
            desired_features (float): desired points for interaction matrix for the waypoint
            waypoint_num (int): current waypoint number

        Returns:
            bool, bool: first is true if converged, false otherwise. second is true if preempted, false otherwise
        """
        features = {}
        feature: DesiredFeature
        for feature in desired_features:
            if features.get(feature.camera_idx, None) is not None:
                features[feature.camera_idx].append([feature.point.x, feature.point.y])
            else:
                features[feature.camera_idx] = [[feature.point.x, feature.point.y]]

        interaction_mat = []
        for key, value in features.items():
            L = self.generate_interaction_matrix(
                desired_points=value, desired_camera_depth=desired_depth, camera_idx=key
            )
            interaction_mat.append(L)
        interaction_mat = np.asarray(interaction_mat)

        r = rospy.Rate(30)  # hz for how frequently we will accept a new velocity
        for i in range(max_it):
            # check for action preemption
            if self.ibvs_waypoint_server.is_preempt_requested():
                rospy.loginfo(f"IBVS preempted on iteration {i}")
                self.move_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # stop moving
                return False, True

            # calculate new velocity, and move.
            vels = self.get_move_vel(
                interaction_mat=interaction_mat, step_size=step_size
            )[0]
            rospy.loginfo(f"VELOCITIES: {vels}")

            # get error norm for action feedback
            feedback: IBVSWaypointFollowFeedback = IBVSWaypointFollowFeedback()
            feedback.remaining_error_norm = np.linalg.norm(self.latest_error)
            feedback.iteration = i
            feedback.waypoint_num = waypoint_num
            self.ibvs_waypoint_server.publish_feedback(feedback)

            self.move_vel(velocities=vels)

            if np.linalg.norm(self.latest_error) < threshold:
                rospy.loginfo(
                    f"IBVS converged to waypoint {waypoint_num} after {i} iterations"
                )
                self.move_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # stop moving
                return True, False

            r.sleep()
            self.move_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # stop moving

        rospy.logwarn(
            f"IBVS did not converge to waypoint {waypoint_num} after {max_it} iterations."
        )
        self.move_vel([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # stop moving
        return False, False


def main(args):
    rospy.sleep(
        3
    )  # this node seems to need a sleep to start properly in tmux, not sure why, TODO: try to fix.
    rospy.loginfo("Starting vs node...")
    node = VisualServoing()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down vs node...")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
