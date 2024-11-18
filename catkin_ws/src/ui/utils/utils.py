from ws_utils.enums import *
from custom_msgs.msg import (
    TrackComponent,
    ErrorDefinition,
    Point2D,
    DistanceDefinition,
    UVSRequestAction,
    UVSRequestGoal,
    DesiredFeature,
    IBVSWaypoint,
)
import customtkinter as ctk
import tkinter as tk
from customtkinter import CTkFont as font
import rospy, typing
from std_msgs.msg import Empty

import os
from kortex_driver.srv import *
from kortex_driver.msg import *
from kortex_bringup import KinovaGen3, KinovaUtil
import numpy as np


def entryIntCallback(P):
    """verifies if input is valid - integers"""
    if str.isdigit(P) or P == "":
        return True
    else:
        return False


def entryFloatCallback(P):
    """verifies is input is valid - floats"""
    if str.isdigit(P) or P == "" or P == ".":
        return True
    else:
        return False


def getFeatures(err, t, idx, depth=0.5):
    points = []

    for p in err.components:
        if p.type == t:
            for i in range(0, len(p.points)):
                x, y = p.points[i].x, p.points[i].y
                points.append([x, y])
    features = []
    for p in points:
        new_point = []
        for i in [[1, 1], [1, -1], [-1, 1], [-1, -1]]:
            d = DesiredFeature()
            d.camera_idx = int(idx)
            point = Point2D()
            point.x = p[0] + (5 * i[0])
            point.y = p[1] + (5 * i[1])
            d.point = point
            new_point.append(d)
        features.append(new_point)

    if t == TrackComponentType.WAYPOINTS:
        ibvs_waypoints = []
        for f in features:
            w = IBVSWaypoint()
            w.desired_features = f
            w.desired_depth = depth
            ibvs_waypoints.append(w)
        return ibvs_waypoints
    else:
        return features[0]


class EStop(ctk.CTkFrame):
    """_summary_
    button widget for determining task types. Extends Frame
    """

    def __init__(self, parent, root):
        """_summary_
                Initializes widget, establishes buttons
        Args:
            parent (_type_): tells widget where to send callbacks
        """
        ctk.CTkFrame.__init__(self, root, fg_color="transparent")
        f = font(family="Helvetica", size=16, weight="normal")

        button = ctk.CTkButton(
            self,
            text="Cancel\nServoing",
            command=self.eStop,
            font=f,
            width=90,
            height=60,
            fg_color="#c42d2d",
            hover_color="#992323",
        )
        button.grid(column=1, row=0, padx=5, pady=5)

        self.grid_columnconfigure((0, 2), weight=1)
        self.parent = parent
        self.grid_rowconfigure((0, 2), weight=1)
        self.exc_cancel = rospy.Publisher("/excavation/cancel", Empty, queue_size=10)

    def eStop(self, *args):
        self.parent.cancelServoing()
        self.exc_cancel.publish(Empty())


class TogglePause(ctk.CTkFrame):
    """_summary_
    button widget for determining task types. Extends Frame
    """

    def __init__(self, parent, root):
        """_summary_
                Initializes widget, establishes buttons
        Args:
            parent (_type_): tells widget where to send callbacks
        """
        ctk.CTkFrame.__init__(self, root, fg_color="transparent")
        f = font(family="Helvetica", size=16, weight="normal")

        self.button = ctk.CTkButton(
            self, text="Pause", font=f, command=self.eStop, width=90, height=60
        )
        self.button.grid(column=1, row=0, sticky="ew", padx=5, pady=5)

        self.parent = parent
        self.text = ["Pause", "Resume"]
        self.state = 0
        self.grid_columnconfigure((0, 2), weight=1)

    def eStop(self, *args):
        self.parent.toggleServoing()
        self.state = 1 - self.state
        self.button.configure(text=self.text[self.state])


class PublisherFromArgs(ctk.CTkButton):
    def __init__(
        self,
        master,
        pub_name: str,
        published_type: typing.Any,
        button_name: str,
        type_args: dict,
        kwargs: dict,
    ):
        super().__init__(
            master=master, text=button_name, command=self.publish_mssg, **kwargs
        )
        self.pub = rospy.Publisher(pub_name, published_type, queue_size=10)

        # self.message = published_type(**type_args)
        self.published_type = published_type
        self.type_args = type_args

    def publish_mssg(self):
        args = {}
        for i in self.type_args:
            if type(self.type_args[i]) in {tk.IntVar, tk.BooleanVar}:
                args[i] = self.type_args[i].get()
            else:
                args[i] = self.type_args[i]
        self.message = self.published_type(**args)
        self.pub.publish(self.message)


class GripperControl(ctk.CTkFrame):
    def __init__(self, parent):
        ctk.CTkFrame.__init__(self, parent)
        title = ctk.CTkLabel(self, text="Gripper")
        self.close = ctk.CTkButton(self, text="Close", command=self.close, width=60)
        self.open = ctk.CTkButton(self, text="Open", command=self.open, width=60)

        self.close.bind("<ButtonRelease>", self.stop)
        self.open.bind("<ButtonRelease>", self.stop)

        title.grid(row=0, columnspan=2, padx=5, pady=5)
        self.close.grid(row=1, column=0, padx=5, pady=5)
        self.open.grid(row=1, column=1, padx=5, pady=5)

        send_gripper_command_full_name = "/my_gen3/base/send_gripper_command"

        self.gripper_command_srv = rospy.ServiceProxy(
            send_gripper_command_full_name, SendGripperCommand
        )

    def close(self):
        self.gripper(-0.3)

    def open(self):
        self.gripper(0.3)

    def stop(self, t):
        self.gripper(0)

    def gripper(self, val):
        # Initialize the request
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = val
        req.input.gripper.finger.append(finger)
        # req.input.mode = GripperMode.GRIPPER_POSITION
        req.input.mode = GripperMode.GRIPPER_SPEED
        # rospy.loginfo("Sending the gripper command: " +  str(value))
        try:
            self.gripper_command_srv(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            # time.sleep(0.01)
            return True


class CartesianControl(ctk.CTkFrame):
    def __init__(self, parent):
        ctk.CTkFrame.__init__(self, parent)
        title = ctk.CTkLabel(self, text="Cartesian Control")
        self.down = ctk.CTkButton(self, text="Down", command=self.Down, width=60)
        self.up = ctk.CTkButton(self, text="Up", command=self.Up, width=60)

        self.down.bind("<ButtonRelease>", self.stop)
        self.up.bind("<ButtonRelease>", self.stop)

        title.grid(row=0, columnspan=2, padx=5, pady=5)
        self.down.grid(row=1, column=0, padx=5, pady=5)
        self.up.grid(row=1, column=1, padx=5, pady=5)
        self.robot_name = "my_gen3"

        self.cartesian_velocity_pub = rospy.Publisher(
            "/" + self.robot_name + "/in/cartesian_velocity",
            TwistCommand,
            queue_size=10,
        )
        play_cartesian_trajectory_full_name = (
            "/" + self.robot_name + "/base/play_cartesian_trajectory"
        )

        self.play_cartesian_trajectory_srv = rospy.ServiceProxy(
            play_cartesian_trajectory_full_name, PlayCartesianTrajectory
        )

    def Down(self):
        self.send_cartesian_pose(-0.05)

    def Up(self):
        self.send_cartesian_pose(+0.05)

    def stop(self, t):
        self.send_cartesian_pose(0)

    def send_cartesian_velocity(self, axes):
        # CARTESIAN_REFERENCE_FRAME_UNSPECIFIED = 0,
        # CARTESIAN_REFERENCE_FRAME_MIXED = 1,
        # CARTESIAN_REFERENCE_FRAME_TOOL = 2,
        # CARTESIAN_REFERENCE_FRAME_BASE = 3,
        cmd = TwistCommand()
        cmd.reference_frame = 0
        cmd.duration = 0
        cmd.twist.linear_x = axes[0] * 0.05
        cmd.twist.linear_y = axes[1] * 0.05
        cmd.twist.linear_z = axes[2] * 0.08
        cmd.twist.angular_x = axes[3] * 0.5
        cmd.twist.angular_y = axes[4] * 0.3
        cmd.twist.angular_z = axes[5] * 0.5
        rospy.loginfo(
            "new cv cmd: [%s, %s, %s, %s, %s, %s]",
            str(cmd.twist.linear_x),
            str(cmd.twist.linear_y),
            str(cmd.twist.linear_z),
            str(cmd.twist.angular_x),
            str(cmd.twist.angular_y),
            str(cmd.twist.angular_z),
        )
        if self.robot_name != "none":
            try:
                self.cartesian_velocity_pub.publish(cmd)
            except rospy.ServiceException:
                rospy.logerr("Failed to publish send_cartesian_velocity")
                return False
            else:
                return True

    def send_cartesian_pose(self, diff):
        if self.robot_name != "none":
            self.last_action_notif_type = None
            # Get the actual cartesian pose to increment it
            # You can create a subscriber to listen to the base_feedback
            # Here we only need the latest message in the topic though
            feedback = rospy.wait_for_message(
                "/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback
            )
            req = PlayCartesianTrajectoryRequest()
            req.input.target_pose.x = feedback.base.commanded_tool_pose_x
            req.input.target_pose.y = feedback.base.commanded_tool_pose_y
            req.input.target_pose.z = feedback.base.commanded_tool_pose_z + diff
            req.input.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x
            req.input.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y
            req.input.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z

            pose_speed = CartesianSpeed()
            pose_speed.translation = 0.1
            pose_speed.orientation = 15
            # The constraint is a one_of in Protobuf. The one_of concept does not exist in ROS
            # To specify a one_of, create it and put it in the appropriate list of the oneof_type member of the ROS object :
            req.input.constraint.oneof_type.speed.append(pose_speed)
            # Call the service
            rospy.loginfo("Sending the robot to the cartesian pose...")
            try:
                self.play_cartesian_trajectory_srv(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call PlayCartesianTrajectory")
                return False


class ThresholdControl(ctk.CTkFrame):
    def __init__(self, root, parent, vals=["10", "20", "30", "40", "50"]):
        ctk.CTkFrame.__init__(self, root)
        title = ctk.CTkLabel(self, text="Threshold")
        title.grid(row=0, columnspan=2, padx=5, pady=5)

        self.threshold = ctk.CTkOptionMenu(
            self, values=vals, command=self.threshold_callback
        )
        self.parent = parent
        twenty = ctk.StringVar(value="20")
        self.threshold.configure(variable=twenty)
        self.threshold.grid(row=1, column=0, padx=5, pady=5)

    def threshold_callback(self):
        self.parent.threshold = int(self.threshold.get())


class RobotInterface(ctk.CTkFrame):
    def __init__(self, root, kinova_present=False):
        ctk.CTkFrame.__init__(self, root)

        if kinova_present:
            self.robot = KinovaGen3()
            self.utils = KinovaUtil()

            # buttons......
            ctk.CTkButton(self, text="Go Home", command=self.kinovaHome).grid(
                row=0, column=0, padx=5, pady=5
            )
            ctk.CTkButton(
                self, text="Record Position", command=self.cachePosition
            ).grid(row=1, column=0, padx=5, pady=5)
            ctk.CTkButton(self, text="Go To Position", command=self.goToPosition).grid(
                row=2, column=0, padx=5, pady=5
            )

    def kinovaHome(self):
        angles = np.array(
            [
                1.28859193,
                0.75965026,
                -3.04099174,
                -1.32838688,
                -0.10518741,
                -0.85721083,
                1.3454415,
            ]
        )
        angles = np.array(
            [
                6.02827235e-02,
                8.91644309e-01,
                3.12935783e00,
                -1.06362553e00,
                2.27007851e-03,
                -1.10575514e00,
                -1.67499570e00,
            ]
        )
        success = self.robot.send_joint_angles(angles)

    def cachePosition(self):
        self.pose = self.utils.get_arm_joints()
        print(self.pose)

    def goToPosition(self):
        self.robot.send_joint_angles(self.pose)
