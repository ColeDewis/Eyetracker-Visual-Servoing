#!/usr/bin/python3

import time
from math import radians, degrees

import numpy as np
import rospy
from sensor_msgs.msg import JointState

from kortex_driver.srv import *
from kortex_driver.msg import *


# #####
# Limits for Gen3 Joints
# In degree

JOINT_LIMIT = {
    0: [radians(-180.0), radians(180.0)],
    1: [radians(-128.9), radians(128.9)],
    2: [radians(-180.0), radians(180.0)],
    3: [radians(-147.8), radians(147.8)],
    4: [radians(-180.0), radians(180.0)],
    5: [radians(-120.3), radians(120.3)],
    6: [radians(-180.0), radians(180.0)],
}

JOINT_NAME_TO_ID = {
    "joint_1": 0,
    "joint_2": 1,
    "joint_3": 2,
    "joint_4": 3,
    "joint_5": 4,
    "joint_6": 5,
    "joint_7": 6,
}

# #####
# Kinova Gen3 Object
# #####


class SimulatedGen3(object):
    """Simulated Gen3."""

    def __init__(
        self,
        robot_name: str = "my_gen3",
        read_joint_state: bool = False,
    ):
        # ####################
        # Connect to Gen3 and setup publishers and subscribers
        # ####################
        try:
            # Gen3 Parameters
            # -----
            self.robot_name = rospy.get_param("~robot_name", robot_name)
            self.dof = 7
            self.joint_names = [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
                "joint_7",
                "finger_joint",
            ]
            self.HOME_ACTION_IDENTIFIER = 2  # The Home Action is used to home the robot. It cannot be deleted and is always ID #2
            self.JOINT_NAME_TO_ID = JOINT_NAME_TO_ID
            self.JOINT_LIMIT = JOINT_LIMIT

            # Gen3 services
            # -----
            self._init_gen3_services()

            # Gen3 subscribers
            # -----
            # Store robot pose into python
            self.position = None
            if read_joint_state:
                self.joint_state_sub = rospy.Subscriber(
                    "/my_gen3/joint_states",
                    JointState,
                    self._joint_state_cb,
                )

            self.cartesian_vel_pub = rospy.Publisher(
                "/gen3/in/cartesian_velocity",
                TwistCommand,
                queue_size=10,
            )

        except rospy.ServiceException:
            rospy.logerr(
                "Failed to initialize Simulated Gen3, {}!".format(self.robot_name)
            )
            rospy.signal_shutdown("Exiting...")

    def _init_gen3_services(self):
        """Initialize Gen3 services."""
        try:
            # For Joint Angles
            # Execute action
            execute_action_full_name = "/gen3/sim/execute_action"
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(
                execute_action_full_name, ExecuteAction
            )

            # For Joint Velocities
            send_joint_speeds_command_full_name = "/gen3/sim/send_joint_speeds_command"
            rospy.wait_for_service(send_joint_speeds_command_full_name)
            self.send_joint_speeds_command = rospy.ServiceProxy(
                send_joint_speeds_command_full_name, SendJointSpeedsCommand
            )

            # For Twist
            send_twist_command_full_name = "/gen3/sim/send_twist_command"
            rospy.wait_for_service(send_twist_command_full_name)
            self.send_twist_command = rospy.ServiceProxy(
                send_twist_command_full_name, SendTwistCommand
            )

            # Gripper command
            send_gripper_command_full_name = "/gen3/sim/send_gripper_command"
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command_srv = rospy.ServiceProxy(
                send_gripper_command_full_name, SendGripperCommand
            )

        except rospy.ServiceException:
            rospy.logerr("Failed to initialize Kinova Gen3 services!")

    def _joint_state_cb(self, msg):
        """Store joint angles inside the class instance."""
        self.position = np.array(msg.position[: len(self.joint_names)]).astype(
            np.float32
        )
        for i in range(len(self.position)):
            self.position[i] = np.clip(
                self.position[i], JOINT_LIMIT[i][0], JOINT_LIMIT[i][1]
            )
        self.vel = np.array(msg.velocity[: len(self.joint_names)]).astype(np.float32)

    def go_home(self):
        """Send Gen3 to default home position."""
        raise NotImplementedError("Simulator does not currently support go home.")

    def send_joint_angles(
        self,
        angles: list,
        angular_duration: float = 0.0,
        MAX_ANGULAR_DURATION: float = radians(30.0),
    ):
        """Move Gen3 to specified joint angles.
        Args:
            angles: list, 7 DOF, in radians.
            angular_duration: float. Control duration between AngularWaypoint
                in a trajectory. 0 by default.
            MAX_ANGULAR_DURATION: float. To validate if angles are safe.
        """
        # Make sure angles is a numpy array
        if isinstance(angles, list):
            angles = np.array(angles)

        # Clip the degrees by joint_limit
        for i in range(len(angles)):
            angles[i] = np.clip(
                angles[i], a_min=JOINT_LIMIT[i][0], a_max=JOINT_LIMIT[i][1]
            )
            angles[i] = degrees(angles[i])

        # Initialization
        self.last_action_notif_type = None
        req = ExecuteActionRequest()
        constrained_angles: ConstrainedJointAngles = ConstrainedJointAngles()

        joint_angles = []
        for angle in angles:
            joint_angles.append(JointAngle(value=angle))
        constrained_angles.joint_angles = JointAngles(joint_angles=joint_angles)
        req.input.oneof_action_parameters.reach_joint_angles = [constrained_angles]

        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointjectory")
            return False
        else:
            return True

    def send_joint_velocities(
        self,
        vels: list,
    ):
        """Set velocity for each individual joint.
        Use rad/s as inputs, then convert to deg/s to Kinova's likings.
        TODO: Might need to change max_rate to suit control frequency.
        """
        # Make sure angles is a numpy array
        if isinstance(vels, list):
            vels = np.array(vels)

        # Initialization
        req = SendJointSpeedsCommandRequest()

        # Joint Speed to send to the robot
        for i in range(len(vels)):
            vel = vels[i]
            joint_speed = JointSpeed()
            joint_speed.joint_identifier = i
            joint_speed.value = vel
            req.input.joint_speeds.append(joint_speed)

        # Send the velocity
        # rospy.loginfo("Sending the joint speed...")

        try:
            self.send_joint_speeds_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendJointSpeedsCommand!")
            return False
        else:
            return True

    def send_twist(
        self,
        vels: list,
        duration_ms: int,
        reference_frame: CartesianReferenceFrame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE,
    ):
        """Sends twist to the robot

        Args:
            vels (list): list of velocities (3 linear, 3 angular). Linear in m/s, angular deg/s
            duration_ms (int): twist command duration (millis)
            reference_frame (CartesianReferenceFrame, optional): reference frame for the twist. Defaults to CARTESIAN_REFERENCE_FRAME_BASE. CURRENTLY ONLY SUPPORTS BASE FRAME.
        """
        req = SendTwistCommandRequest()
        twist_command = TwistCommand()
        twist_command.duration = (
            duration_ms  # TODO: documentation seems to suggest this doesn't do anything
        )
        twist_command.reference_frame = reference_frame
        twist = Twist()
        twist.linear_x = vels[0]
        twist.linear_y = vels[1]
        twist.linear_z = vels[2]
        twist.angular_x = vels[3]
        twist.angular_y = vels[4]
        twist.angular_z = vels[5]
        twist_command.twist = twist
        req.input = twist_command

        # rospy.loginfo(f"Sending twist with reference frame {reference_frame} and duration {duration_ms}")

        try:
            self.send_twist_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendTwistCommand!")
            return False
        else:
            return True

    def send_twist_topic(
        self,
        vels: list,
        duration_ms: int,
        reference_frame: CartesianReferenceFrame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE,
    ):
        """Sends twist to the robot, but using the /in topic instead of a service

        Args:
            vels (list): list of velocities (3 linear, 3 angular). Linear in m/s, angular deg/s
            duration_ms (int): twist command duration (millis)
            reference_frame (CartesianReferenceFrame, optional): reference frame for the twist. Defaults to CARTESIAN_REFERENCE_FRAME_BASE.
        """
        req = SendTwistCommandRequest()
        twist_command = TwistCommand()
        twist_command.duration = (
            duration_ms  # TODO: documentation seems to suggest this doesn't do anything
        )
        twist_command.reference_frame = reference_frame
        twist = Twist()
        twist.linear_x = vels[0]
        twist.linear_y = vels[1]
        twist.linear_z = vels[2]
        twist.angular_x = vels[3]
        twist.angular_y = vels[4]
        twist.angular_z = vels[5]
        twist_command.twist = twist

        self.cartesian_vel_pub.publish(twist_command)

    def send_pose(
        self, x, y, z, theta_x, theta_y, theta_z, trans_speed_limit, rot_speed_limit
    ):
        """NOTE: IN SIMULATION, ONLY THE TRANSLATION WILL BE UPDATED, THE ORIENTATION WILL BE IGNORED.

        Sends a 3d pose to the gen3 relative to the base link

        Args:
            x (float): x target in m
            y (float): y target in m
            z (float): z target in m
            theta_x (float): x angle target in degrees
            theta_y (float): y angle target in degrees
            theta_z (float): z angle target in degrees
            trans_speed_limit (float): translational speed limit in m/s
            rot_speed_limit (float): rotational speed limit in deg/s

        Returns:
            bool: true/false depending on whether the command succeeded
        """
        """Sends a 3d pose to the gen3 relative to the base link

        Args:
            x (float): x target in m
            y (float): y target in m
            z (float): z target in m
            theta_x (float): x angle target in degrees
            theta_y (float): y angle target in degrees
            theta_z (float): z angle target in degrees
            trans_speed_limit (float): translational speed limit in m/s
            rot_speed_limit (float): rotational speed limit in deg/s

        Returns:
            bool: true/false depending on whether the command succeeded
        """
        my_cartesian_speed = CartesianSpeed()
        my_cartesian_speed.translation = trans_speed_limit  # m/s
        my_cartesian_speed.orientation = rot_speed_limit  # deg/s

        my_constrained_pose = ConstrainedPose()
        my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)

        my_constrained_pose.target_pose.x = x
        my_constrained_pose.target_pose.y = y
        my_constrained_pose.target_pose.z = z
        my_constrained_pose.target_pose.theta_x = theta_x
        my_constrained_pose.target_pose.theta_y = theta_y
        my_constrained_pose.target_pose.theta_z = theta_z

        req = ExecuteActionRequest()
        req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
        req.input.name = "pose_move"
        req.input.handle.action_type = ActionType.REACH_POSE
        req.input.handle.identifier = 1001

        rospy.loginfo("Sending pose...")
        self.last_action_notif_type = None
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to send pose")
            return False
        else:
            rospy.loginfo("Waiting for pose to finish...")

        return True

    def send_gripper_command(
        self,
        value: float,
    ):
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service
        try:
            self.send_gripper_command_srv(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True

    def __str__(self):
        string = "Kinova Gen3\n"
        string += "  robot_name: {}\n  dof: {}".format(self.robot_name, self.dof)
        return string
