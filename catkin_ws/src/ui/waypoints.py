import tkinter as tk
import customtkinter as ctk
from tkinter import ttk
from ws_utils.enums import TrackComponentType, ErrorDefinitionType
from utils.utils import *

global TRACKERTYPEDICT
TRACKERTYPEDICT = {
    "Fixed": TrackerType.FIXED,
    "LK-Pyr": TrackerType.LKPYR,
    "Segment": TrackerType.SEGMENTATION,
    "CAMshift": TrackerType.CAMSHIFT,
    "Segment-CAMshift-Layered": TrackerType.SEGCAMSHIFTLAYERED,
}


class WaypointTab(ctk.CTkFrame):
    def __init__(self, parent, root):
        ctk.CTkFrame.__init__(self, root)
        self._fg_color = "transparent"

        self.start_goal = StartGoal(parent, root)
        self.place_waypoints = PlaceWaypoints(parent, root)
        # self.interpolation = Interpolation(parent, root)
        self.done = Done(parent, root)
        self.stop = EStop(parent, root)

        self.start_goal.grid(row=0, column=0, sticky="new", padx=10, pady=10)
        self.place_waypoints.grid(row=0, column=1, sticky="new", padx=10, pady=10)
        # self.interpolation.grid(row=0, column=2, sticky="new", padx=10, pady=10)
        self.done.grid(row=0, column=3, sticky="new", padx=10, pady=10)
        self.stop.grid(row=0, column=4, sticky="new", padx=10, pady=10)


class StartGoal(ctk.CTkFrame):
    """_summary_
    Button widget for determining tracker type. extends frame
    """

    def __init__(self, parent, root):
        """_summary_
                Initializes widget, establishes buttons
        Args:
            parent (_type_): tells widget where to send callbacks
        """
        ctk.CTkFrame.__init__(self, root)
        self._fg_color = "transparent"
        global TRACKERTYPEDICT

        ctk.CTkLabel(self, text="Place Initial Point").grid(
            column=0, row=0, columnspan=2, sticky="ew", padx=5, pady=5
        )

        self.zoomed = tk.IntVar()
        ctk.CTkCheckBox(self, text="Zoom in", variable=self.zoomed).grid(
            column=0, row=1, sticky="ew", padx=5, pady=5, columnspan=2
        )

        ctk.CTkButton(self, text="Place", command=self.initialPoint, width=80).grid(
            column=0, row=2, sticky="ew", padx=5, pady=5
        )

        self.t_type = ctk.CTkOptionMenu(
            self, values=list(TRACKERTYPEDICT.keys()), width=80
        )
        self.t_type.grid(column=1, row=2, sticky="ew")

        self.parent = parent

    def initialPoint(self, *args):
        self.parent.placeTracker(
            1,
            TrackComponentType.ANY_POINT,
            TRACKERTYPEDICT[self.t_type.get()],
            self.zoomed.get(),
        )


class PlaceWaypoints(ctk.CTkFrame):
    """_summary_
    Button widget for determining tracker type. extends frame
    """

    def __init__(self, parent, root):
        """_summary_
                Initializes widget, establishes buttons
        Args:
            parent (_type_): tells widget where to send callbacks
        """
        ctk.CTkFrame.__init__(self, root)
        self._fg_color = "transparent"

        ctk.CTkLabel(self, text="Number of Waypoints").grid(
            column=0, row=0, columnspan=2, sticky="ew", padx=5, pady=5
        )

        self.int_callback = self.register(entryIntCallback)
        self.n_waypoints = ctk.CTkEntry(
            self, width=5, validate="all", validatecommand=(self.int_callback, "%P")
        )
        self.n_waypoints.grid(column=0, row=1, sticky="ew", padx=5, pady=5)

        ctk.CTkButton(self, text="Place Waypoints", command=self.placeWaypoints).grid(
            column=0, row=2, sticky="ew", padx=5, pady=5
        )

        self.parent = parent

    def placeWaypoints(self, *args):
        self.parent.placeTracker(
            int(self.n_waypoints.get()),
            TrackComponentType.WAYPOINTS,
            TrackerType.FIXED,
            False,
        )


class Interpolation(ctk.CTkFrame):
    """_summary_
    Button widget for determining tracker type. extends frame
    """

    def __init__(self, parent, root):
        """_summary_
                Initializes widget, establishes buttons
        Args:
            parent (_type_): tells widget where to send callbacks
        """
        ctk.CTkFrame.__init__(self, root)
        self._fg_color = "transparent"

        ctk.CTkLabel(self, text="Interpolate").grid(
            column=0, row=0, sticky="ew", padx=5, pady=5
        )

        self.int_callback = self.register(entryIntCallback)
        self.n_waypoints = ctk.CTkEntry(
            self, width=5, validate="all", validatecommand=(self.int_callback, "%P")
        )

        self.n_waypoints.grid(column=0, row=1, sticky="ew", padx=5, pady=5)

        ctk.CTkButton(self, text="Interpolate", command=self.interpolate).grid(
            column=0, row=2, sticky="ew", padx=5, pady=5
        )

        self.parent = parent

    def interpolate(self, *args):
        self.parent.placeTracker(
            int(self.n_waypoints.get()), TrackComponentType.WAYPOINTS, self.zoomed.get()
        )


class Done(ctk.CTkFrame):
    """_summary_
    Button widget for managing utils, like initialize, reset, and go. Extends frame
    """

    def __init__(self, parent, root):
        """_summary_
                Initializes widget, establishes buttons
        Args:
            parent (_type_): tells widget where to send callbacks
        """
        ctk.CTkFrame.__init__(self, root)
        self.not_cam_2 = tk.IntVar()
        self.not_cam_1 = tk.IntVar()

        # ctk.CTkButton(self, text="Initialize", command=self.init).grid(column=1, row=0, columnspan = 2, sticky="ew",padx=5,pady=5)
        ctk.CTkButton(self, text="Init - UVS", command=self.initUVS, width=20).grid(
            column=1, row=0, sticky="ew", padx=5, pady=5
        )
        ctk.CTkButton(self, text="Init - IBVS", command=self.initIBVS, width=20).grid(
            column=2, row=0, sticky="ew", padx=5, pady=5
        )

        ctk.CTkButton(self, text="Reset", command=self.reset).grid(
            column=1, row=1, columnspan=2, sticky="ew", padx=5, pady=5
        )

        # ctk.CTkButton(self, text="Go - UVS", command=self.goUVS, width=20).grid(column=1, row=2, sticky="ew",padx=5,pady=5)
        # ctk.CTkButton(self, text="Go - IBVS", command=self.goIBVS, width=20).grid(column=2, row=2, sticky="ew",padx=5,pady=5)
        ctk.CTkButton(self, text="Go ", command=self.go, width=20).grid(
            column=1, row=2, columnspan=2, sticky="ew", padx=5, pady=5
        )

        ctk.CTkCheckBox(self, text="Cam 1 Only", variable=self.not_cam_2).grid(
            column=0, row=1, sticky="ew", padx=5, pady=5
        )
        ctk.CTkCheckBox(self, text="Cam 2 Only", variable=self.not_cam_1).grid(
            column=0, row=2, sticky="ew", padx=5, pady=5
        )

        self.parent = parent

    def initUVS(self, *args):
        self.parent.initTrackers(
            "uvsWaypoint", self.not_cam_2.get(), self.not_cam_1.get()
        )

    def initIBVS(self, *args):
        self.parent.initTrackers(
            "ibvsWaypoint", self.not_cam_2.get(), self.not_cam_1.get()
        )

    def reset(self, *args):
        self.parent.resetAll()

    def go(self, *args):
        self.parent.go(self.not_cam_2.get(), self.not_cam_1.get())
