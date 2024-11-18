import customtkinter as ctk

import sensor_msgs.msg as ms
from std_msgs.msg import Empty, Int32


from button_widgets import *
from camera_widget import *
from waypoints import *
from camera_position import *
from misc import *

import rospy
import uuid

from custom_msgs.msg import (
    ErrorDefinition,
    DistanceDefinition,
    UVSRequestAction,
    UVSRequestGoal,
)
import actionlib
from custom_msgs.msg import UVSRequestAction, UVSRequestGoal
from custom_msgs.msg import IBVSRequestAction, IBVSRequestGoal
from custom_msgs.msg import (
    UVSWaypointFollowAction,
    UVSWaypointFollowFeedback,
    UVSWaypointFollowGoal,
)
from custom_msgs.msg import (
    IBVSWaypointFollowAction,
    IBVSWaypointFollowFeedback,
    IBVSWaypointFollowGoal,
)
from custom_msgs.msg import ExcavationFeedback


from ws_utils.enums import *

import faulthandler
from threading import *

import argparse


class TabManager(ctk.CTkTabview):
    def __init__(self, parent, kinova_present=False):
        ctk.CTkTabview.__init__(self, parent)
        f = font(family="Helvetica", size=16, weight="normal")
        self._segmented_button.configure(font=f)
        self.kinova_present = kinova_present

        pos = "Geometric"
        vel = "Waypoints"
        cam = "Camera Position"
        scr = "Scripts"
        misc = "IDEaS Demo"

        self.add(pos)
        self.add("Positions")
        self.add(vel)
        self.add(cam)
        self.add(misc)

        self.pos = RobotInterface(self.tab("Positions"), self.kinova_present)
        self.pos.grid(row=1, column=0, sticky="ew")
        self.pos_widget = ButtonTab(parent, self.tab(pos))
        self.vel_widget = WaypointTab(parent, self.tab(vel))
        self.cam_pos_widget = CameraPositionTab(parent, self.tab(cam))
        self.joystick_widget = ScriptJoyTab(parent, self.tab(misc), self.kinova_present)

        self.pos_widget.grid(column=0, row=0, sticky="nesw", padx=5, pady=5)
        self.vel_widget.grid(column=0, row=0, sticky="nesw", padx=5, pady=5)
        self.cam_pos_widget.grid(column=0, row=0, sticky="nesw", padx=5, pady=5)
        self.joystick_widget.grid(
            column=0, row=0, columnspan=4, sticky="nesw", padx=5, pady=5
        )

        self.set(pos)
        self.tab(misc).grid_columnconfigure((0), weight=1)
        self.tab(pos).grid_columnconfigure((0), weight=1)


class UI(ctk.CTkFrame):
    """_summary_

    Main widget which drives and handles others

    """

    def __init__(self, parent, kinova=False):
        """_summary_

            Lays out all other widgets, and sets up instances for other widgets

        Args:
            parent (_type_): _description_
        """
        ctk.CTkFrame.__init__(self, parent)
        self.kinova_present = kinova
        self._fg_color = "transparent"
        rospy.init_node("ui_node")

        self.camIndices = []

        # cam_idx_thread
        self.cam_idx_thread = Thread(target=self.getNewCams)
        self.cam_idx_thread.start()

        # cam_idx_sub = rospy.Subscriber("/cam_idx", Int32, self.idx_cb)
        while False:  # len(self.camIndices) < 2:
            continue

        rospy.loginfo(f"Cam Indices: {self.camIndices}")

        self.tab_manager = TabManager(self, self.kinova_present)

        self.tab_manager.grid(row=0, column=0, columnspan=4, sticky="new")
        # self.tab_manager.grid_columnconfigure(0, weight=1)

        self.cam1_thread = Thread(target=self.cam1_setup)
        self.cam1_thread.start()
        self.cam2_thread = Thread(target=self.cam2_setup)
        self.cam2_thread.start()

        self.info = ctk.CTkFrame(self)
        self.info.grid(row=2, column=0, columnspan=4, sticky="nw")

        self.status = ctk.CTkLabel(self.info, text="Servoing: Not Started")
        self.status.grid(row=0, column=0, sticky="nw")
        self.error = ctk.CTkLabel(self.info, text="Error: ")
        self.error.grid(row=1, column=0, sticky="nw")
        self.waypoint_num = ctk.CTkLabel(self.info, text="")
        self.waypoint_num.grid(row=2, column=0, sticky="nw")

        self.error_req_pub = rospy.Publisher(
            "/tracking_node/error_request", ErrorDefinition, queue_size=10
        )
        self.vs_start_pub = rospy.Publisher("/vs_start", Empty, queue_size=10)
        self.trackers_remove_pub = rospy.Publisher(
            "/tracking_node/reset_all", Empty, queue_size=10
        )
        self.error_req1 = ErrorDefinition()
        self.error_req2 = ErrorDefinition()
        self.distance1 = DistanceDefinition()
        self.distance2 = DistanceDefinition()
        self.client = None
        self.task_types = []

        self.joystick_updater_sub = None
        self.scriptName = None
        self.threshold = 20

    def cam1_setup(self):
        self.cam1 = CameraDisplays(self, self.camIndices[0])
        self.cam1.grid(row=1, column=0, columnspan=2, sticky="new")

    def cam2_setup(self):
        self.cam2 = CameraDisplays(self, self.camIndices[-1])
        self.cam2.grid(row=1, column=2, columnspan=2, sticky="new")

    def getNewCams(self):
        cam_idx_sub = rospy.Subscriber("/cam_idx", Int32, self.idx_cb)

    def idx_cb(self, data):
        """_summary_

            Pulls out camera indices for running video feed.

        Args:
            data (_type_): message which contains cam_idx
        """
        if data.data not in self.camIndices:
            self.camIndices.append(data.data)

    def getCamIds(self):
        return list(map(str, self.camIndices))

    # TODO this should be in button widget
    def defineTask(self, id1):
        """_summary_

            Callback function for setting up error requests with correct task info

        Args:
            id (_type_): type of task
        """

        self.error_req1.type = id1
        self.error_req2.type = id1
        self.error_req1.task_scale = 1.0
        self.error_req2.task_scale = 1.0
        self.error_req1.cam_idx = self.camIndices[0]
        self.error_req2.cam_idx = self.camIndices[-1]
        self.error_req1.components = []
        self.error_req2.components = []
        rospy.loginfo(self.error_req2)

    def placeTracker(self, pts, t, t_type, zoom, seg_color=None):
        """_summary_

            Gets camera widgets to go into placing points mode
        Args:
            pts (_type_): number of points to place
            t (_type_): type of tracker
            zoom (_type_): whether or not the user wants to zoom for these points
        """
        notcam2, notcam1 = self.tab_manager.pos_widget.done.getCamStatus()
        if not notcam1:
            self.cam1.pointPlace(zoom, pts, self.error_req1, t, t_type, seg_color)
        if not notcam2:
            self.cam2.pointPlace(zoom, pts, self.error_req2, t, t_type, seg_color)
        if t == TrackComponentType.WAYPOINTS:
            self.error_req1.id = uuid.uuid4().hex
            self.error_req2.id = uuid.uuid4().hex
            self.error_req1.type = ErrorDefinitionType.WAYPOINTS
            self.error_req2.type = ErrorDefinitionType.WAYPOINTS
            self.error_req1.cam_idx = self.camIndices[0]
            self.error_req2.cam_idx = self.camIndices[-1]

    def distanceBaseline(self):
        """_summary_
        Place points for the distance baselines
        """
        self.cam1.pointPlace(True, 4, self.distance1)
        self.cam2.pointPlace(True, 4, self.distance2)

    def requestCompass(self, idx):
        points = (
            self.distance1.plane_points if idx == 0 else self.distance2.plane_points
        )

        print(points)

        m1 = (points[1].y - points[0].y) / (points[1].x - points[0].x)
        m2 = (points[2].y - points[1].y) / (points[2].x - points[1].x)

        l1 = np.sqrt(
            (points[1].y - points[0].y) ** 2 + (points[1].x - points[0].x) ** 2
        )
        l2 = np.sqrt(
            (points[2].y - points[1].y) ** 2 + (points[2].x - points[1].x) ** 2
        )

        if idx == 0:
            self.cam1.calculateCompass(m1, m2, l1, l2)
        else:
            self.cam2.calculateCompass(m1, m2, l1, l2)

    def resetDistance(self):
        self.distance1 = DistanceDefinition()
        self.distance2 = DistanceDefinition()

    def initTrackers(self, type, not_cam_2, not_cam_1):
        """_summary_

            Initializes trackers by publishing tracker information to the tracking node.

        Args:
            not_cam_2 (_type_): True if we don't want to use cam 2
            not_cam_1 (_type_): True if we don't want to use cam 1
        """
        rospy.loginfo("Sending error info stuff")

        if type == "ibvs" and self.error_req1.type == ErrorDefinitionType.POINT_POINT:
            self.error_req1.type = ErrorDefinitionType.IBVS_PT_PT
            self.error_req2.type = ErrorDefinitionType.IBVS_PT_PT
        elif type == "ibvsWaypoint":
            self.error_req1.type = ErrorDefinitionType.IBVS_WAYPOINTS
            self.error_req2.type = ErrorDefinitionType.IBVS_WAYPOINTS

        u, v = self.tab_manager.pos_widget.distance.getDims()

        self.distance1.reference_distance_u = u
        self.distance1.reference_distance_v = v
        self.distance2.reference_distance_u = u
        self.distance2.reference_distance_v = v

        dis, dir1, dir2 = self.tab_manager.pos_widget.distance.getDisDir()

        self.distance1.desired_distance = dis
        self.distance1.direction = dir1
        self.distance2.desired_distance = dis
        self.distance2.direction = dir2

        # publish error requests
        self.error_req1.distance_info = self.distance1
        self.error_req2.distance_info = self.distance2

        if not not_cam_2:
            self.error_req_pub.publish(self.error_req2)
            rospy.logwarn(self.error_req2)
        if not not_cam_1:
            self.error_req_pub.publish(self.error_req1)
            rospy.logwarn(self.error_req1)

        self.task_types.append(self.error_req1.type)

        self.error_req1 = ErrorDefinition()
        self.error_req2 = ErrorDefinition()

        self.cam1.tracks(self.camIndices[0])
        self.cam2.tracks(self.camIndices[-1])

    def resetAll(self):
        """_summary_
        Resets established trackers so they can be placed differently
        """
        rospy.loginfo(self.error_req1.components)
        rospy.loginfo(self.error_req2.components)
        self.error_req1 = ErrorDefinition()
        self.error_req2 = ErrorDefinition()

        self.cam1.reset(self.camIndices[0])
        self.cam2.reset(self.camIndices[-1])
        self.cam1.mssg = None
        self.cam2.mssg = None
        rospy.loginfo("Removing all tracking")
        self.trackers_remove_pub.publish(Empty())

    def go(self, not_cam_2, not_cam_1):
        """Visual servo "go" button, sends vs_start command."""
        print(self.task_types)
        for action in self.task_types:
            if action in (
                ErrorDefinitionType.POINT_POINT,
                ErrorDefinitionType.POINT_LINE,
                ErrorDefinitionType.LINE_LINE,
                ErrorDefinitionType.POINT_CONIC,
            ):
                self.client = actionlib.SimpleActionClient(
                    "/visual_servo_node/uvs_action", UVSRequestAction
                )
                self.client.wait_for_server()
                goal = UVSRequestGoal(
                    learn_rate=0.5,
                    init_step_size=0.1,
                    max_step_size=0.2,
                    max_it=100,
                    threshold=self.threshold,
                    num_joints=4,
                )

            elif action == ErrorDefinitionType.IBVS_PT_PT:
                # try to get the camera position:
                self.tab_manager.cam_pos_widget.publishCamPos(not_cam_2, not_cam_1)

                self.client = actionlib.SimpleActionClient(
                    "/visual_servo_node/ibvs_action", IBVSRequestAction
                )
                self.client.wait_for_server()
                print(self.camIndices, type(self.camIndices))

                if not not_cam_2:
                    features = getFeatures(
                        self.error_req2,
                        TrackComponentType.ANY_POINT,
                        self.camIndices[1],
                    )
                if not not_cam_1:
                    features = getFeatures(
                        self.error_req2,
                        TrackComponentType.ANY_POINT,
                        self.camIndices[0],
                    )

                goal = IBVSRequestGoal(
                    step_size=0.2,
                    max_it=100,
                    threshold=10,
                    desired_depth=0.5,
                    desired_features=features,
                )

            elif action == ErrorDefinitionType.WAYPOINTS:
                self.client = actionlib.SimpleActionClient(
                    "/visual_servo_node/uvs_waypoint_follow", UVSWaypointFollowAction
                )
                self.client.wait_for_server()
                ids = []
                if not not_cam_2:
                    ids.append(self.error_req2.id)
                if not not_cam_1:
                    ids.append(self.error_req1.id)
                ids = []

                goal = UVSWaypointFollowGoal(
                    learn_rate=0.5,
                    init_step_size=0.1,
                    max_step_size=0.2,
                    max_it=100,
                    threshold=10,
                    num_joints=4,
                    waypoint_ids=ids,
                )

            elif action == ErrorDefinitionType.IBVS_WAYPOINTS:
                # try to get the camera position:
                self.publishCamPos(not_cam_2, not_cam_1)

                self.client = actionlib.SimpleActionClient(
                    "/visual_servo_node/ibvs_waypoint_follow", IBVSWaypointFollowAction
                )
                self.client.wait_for_server()
                ids = []
                if not not_cam_2:
                    ids.append(self.error_req2.id)
                    features = getFeatures(
                        self.error_req2,
                        TrackComponentType.WAYPOINTS,
                        self.camIndices[1],
                    )
                if not not_cam_1:
                    ids.append(self.error_req1.id)
                    features = getFeatures(
                        self.error_req2,
                        TrackComponentType.WAYPOINTS,
                        self.camIndices[0],
                    )

                goal = IBVSWaypointFollowGoal(
                    step_size=0.2,
                    max_it=100,
                    threshold=10,
                    ibvs_waypoints=features,
                    waypoint_ids=ids,
                )
            self.client.send_goal(
                goal, feedback_cb=self.callback_feedback, done_cb=self.callback_done
            )
            self.status.configure(text="Servoing: In progress")

    def callback_feedback(self, feedback):
        rospy.loginfo("Feedback:%s" % str(feedback))
        self.error.configure(text="Error: " + str(feedback.remaining_error_norm))
        if (
            type(feedback) == UVSWaypointFollowFeedback
            or type(feedback) == IBVSWaypointFollowFeedback
        ):
            self.waypoint_num.configure(text="Waypoint: " + str(feedback.waypoint_num))

    def callback_done(self, state, result):
        rospy.loginfo(f"done called {result}")
        if result.converged:
            self.status.configure(text="Status: Servoing converged")
        else:
            self.status.configure(text="Status: Servoing could not converge")

    def cancelServoing(self):
        if self.client is not None:
            self.client.cancel_all_goals()

    def grabImage(self, cam_num):
        if cam_num == self.camIndices[0]:
            return self.cam1.img
        else:
            return self.cam2.img


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-k", "--Kinova", action="store_true")
    args = parser.parse_args()
    faulthandler.enable()
    ctk.set_appearance_mode("System")  # Modes: system (default), light, dark
    ctk.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green
    root = ctk.CTk()
    root.winfo_name = ""
    root.minsize(1350, 870)
    print(args)
    UI(root, args.Kinova).place(relx=0.5, rely=0, anchor="n")
    root.mainloop()
