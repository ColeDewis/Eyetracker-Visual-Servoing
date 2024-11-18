import tkinter as tk
import customtkinter as ctk
from tkinter import ttk
from ws_utils.enums import (
    TrackComponentType,
    ErrorDefinitionType,
    TrackerType,
    SegColor,
)
from utils.utils import *

global TRACKERTYPEDICT
TRACKERTYPEDICT = {
    "Fixed": TrackerType.FIXED,
    "LK-Pyr": TrackerType.LKPYR,
    "Segment": TrackerType.SEGMENTATION,
    "CAMshift": TrackerType.CAMSHIFT,
    "Segment-CAMshift-Layered": TrackerType.SEGCAMSHIFTLAYERED,
}

global COLORDICT
COLORDICT = {"Green": SegColor.SHOVEL_GREEN, "Orange": SegColor.SHOVEL_ORANGE}


class ButtonTab(ctk.CTkFrame):
    def __init__(self, parent, root):
        ctk.CTkFrame.__init__(self, root)

        self.tasks = TaskType(parent, self)
        self.trackers = TrackerType(parent, self)
        self.distance = DistanceDef(parent, self)
        self.done = Done(parent, self)
        self.stop = EStop(parent, self)

        self.tasks.grid(row=0, column=1, sticky="new", padx=10, pady=10)
        self.trackers.grid(row=0, column=2, sticky="new", padx=10, pady=10)
        self.distance.grid(row=0, column=3, sticky="new", padx=10, pady=10)
        self.done.grid(row=0, column=4, sticky="new", padx=10, pady=10)
        self.stop.grid(row=0, column=5, sticky="new", padx=10, pady=10)
        self.aux_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.gripper = GripperControl(self.aux_frame)
        self.gripper.grid(row=0, column=0)

        self.up_down = CartesianControl(self.aux_frame)
        self.up_down.grid(row=1, column=0)

        self.threshold = ThresholdControl(self.aux_frame, parent)
        self.threshold.grid(row=2, column=0)
        self.aux_frame.grid(row=0, column=6)

        self.grid_columnconfigure((0, 7), weight=1)

    def enableTasks(self, on_off):
        self.tasks.enableTasks(on_off)

    def enableTrackers(self, task_type):
        self.trackers.enableTrackers(task_type)


""" alrighty just heres a plan : 
start with regular task flow:
only tasks available -> only trackers available -> only initialize available -> loop (ish)"""


class TaskType(ctk.CTkFrame):
    """_summary_
    button widget for determining task types. Extends Frame
    """

    def __init__(self, parent, root, fg="transparent"):
        """_summary_
                Initializes widget, establishes buttons
        Args:
            parent (_type_): tells widget where to send callbacks
        """
        ctk.CTkFrame.__init__(self, root, fg_color=fg)
        ctk.CTkLabel(self, text="Task Type").grid(
            column=0, row=0, columnspan=2, sticky="new"
        )

        self.pp = ctk.CTkButton(self, text="Point to Point", command=self.ptp, width=20)
        self.pp.grid(column=0, row=1, sticky="new", padx=5, pady=5)
        self.pl = ctk.CTkButton(self, text="Point to Line", command=self.ptl, width=20)
        self.pl.grid(column=0, row=2, sticky="new", padx=5, pady=5)

        self.ll = ctk.CTkButton(self, text="Line to Line", command=self.ltl, width=20)
        self.ll.grid(column=1, row=1, sticky="new", padx=5, pady=5)
        self.pe = ctk.CTkButton(
            self, text="Point to Ellipse", command=self.pte, width=20
        )
        self.pe.grid(column=1, row=2, sticky="new", padx=5, pady=5)

        self.parent = parent
        self.root = root

    def ptp(self, *args):
        self.parent.defineTask(ErrorDefinitionType.POINT_POINT)
        self.root.enableTrackers(ErrorDefinitionType.POINT_POINT)
        self.enableTasks(False)

    def ptl(self, *args):
        self.parent.defineTask(ErrorDefinitionType.POINT_LINE)
        self.root.enableTrackers(ErrorDefinitionType.POINT_LINE)
        self.enableTasks(False)

    def ltl(self, *args):
        self.parent.defineTask(ErrorDefinitionType.LINE_LINE)
        self.root.enableTrackers(ErrorDefinitionType.LINE_LINE)
        self.enableTasks(False)

    def pte(self, *args):
        self.parent.defineTask(ErrorDefinitionType.POINT_CONIC)
        self.root.enableTrackers(ErrorDefinitionType.POINT_CONIC)
        self.enableTasks(False)

    def enableTasks(self, on_off):
        if on_off:
            for b in [self.pp, self.pl, self.ll, self.pe]:
                b.configure(state="normal")

        else:
            for b in [self.pp, self.pl, self.ll, self.pe]:
                b.configure(state="disabled")


class TrackerType(ctk.CTkFrame):
    """_summary_
    Button widget for determining tracker type. extends frame
    """

    def __init__(self, parent, root, fg="transparent"):
        """_summary_
                Initializes widget, establishes buttons
        Args:
            parent (_type_): tells widget where to send callbacks
        """
        ctk.CTkFrame.__init__(self, root, fg_color=fg)
        self.zoomed = tk.IntVar()
        ctk.CTkLabel(self, text="Tracker Type").grid(column=0, row=0, sticky="new")
        ctk.CTkCheckBox(self, text="Zoom in", variable=self.zoomed).grid(
            column=1, row=0, sticky="new", padx=5, pady=5
        )
        global TRACKERTYPEDICT
        global COLORDICT

        self.p = ctk.CTkButton(self, text="Point", command=self.point, width=20)
        self.p.grid(column=0, row=1, sticky="new", padx=5, pady=5)
        self.p_type = ctk.CTkOptionMenu(
            self, values=list(TRACKERTYPEDICT.keys()), width=20
        )
        self.p_type.grid(column=1, row=1, sticky="new", padx=5, pady=5)
        self.color = ctk.CTkOptionMenu(self, values=list(COLORDICT.keys()), width=20)
        self.color.grid(column=2, row=1, sticky="new", padx=5, pady=5)

        self.l = ctk.CTkButton(self, text="Line", command=self.line, width=20)
        self.l.grid(column=0, row=2, sticky="new", padx=5, pady=5)
        self.l_type = ctk.CTkOptionMenu(
            self, values=list(TRACKERTYPEDICT.keys()), width=20
        )
        self.l_type.grid(column=1, row=2, sticky="new", padx=5, pady=5)

        self.e = ctk.CTkButton(self, text="Ellipse", command=self.ellipse, width=20)
        self.e.grid(column=0, row=3, sticky="new", padx=5, pady=5)
        self.e_type = ctk.CTkOptionMenu(
            self, values=list(TRACKERTYPEDICT.keys()), width=20
        )
        self.e_type.grid(column=1, row=3, sticky="new", padx=5, pady=5)
        self.parent = parent

    def point(self, *args):
        self.parent.placeTracker(
            1,
            TrackComponentType.ANY_POINT,
            TRACKERTYPEDICT[self.p_type.get()],
            self.zoomed.get(),
            COLORDICT[self.color.get()],
        )

    def line(self, *args):
        self.parent.placeTracker(
            2,
            TrackComponentType.ANY_LINE,
            TRACKERTYPEDICT[self.l_type.get()],
            self.zoomed.get(),
        )

    def ellipse(self, *args):
        self.parent.placeTracker(
            5,
            TrackComponentType.CONIC,
            TRACKERTYPEDICT[self.e_type.get()],
            self.zoomed.get(),
        )

    def enableTrackers(self, tracker_type):
        if tracker_type == False:
            for b in [self.p, self.l, self.e]:
                b.configure(state="disabled")
        elif tracker_type == True:
            for b in [self.p, self.l, self.e]:
                b.configure(state="normal")
        else:
            if tracker_type == ErrorDefinitionType.POINT_POINT:
                for b in [self.l, self.e]:
                    b.configure(state="disabled")
            elif tracker_type == ErrorDefinitionType.POINT_LINE:
                for b in [self.e]:
                    b.configure(state="disabled")
            elif tracker_type == ErrorDefinitionType.LINE_LINE:
                for b in [self.p, self.e]:
                    b.configure(state="disabled")
            elif tracker_type == ErrorDefinitionType.POINT_CONIC:
                for b in [self.l]:
                    b.configure(state="disabled")


class DistanceDef(ctk.CTkFrame):
    """_summary_
    Button widget for managing distance definition. Extends frame
    """

    def __init__(self, parent, root, fg="transparent"):
        """_summary_
                Initializes widget, establishes buttons
        Args:
            parent (_type_): tells widget where to send callbacks
        """
        ctk.CTkFrame.__init__(self, root, fg_color=fg)
        ctk.CTkLabel(self, text="Initialize Distance").grid(
            column=0, row=0, columnspan=4, sticky="new", padx=5, pady=5
        )

        ctk.CTkButton(
            self,
            text="Place",
            command=self.place,
            width=80,
        ).grid(column=0, row=1, columnspan=2, sticky="new", padx=5, pady=5)
        ctk.CTkButton(self, text="Reset", width=160, command=self.reset).grid(
            column=0, row=4, columnspan=4, sticky="new", padx=5, pady=5
        )
        ctk.CTkLabel(
            self,
            text="X Dist",
        ).grid(column=0, row=2, sticky="new", padx=5, pady=5)
        ctk.CTkLabel(
            self,
            text="Y Dist",
        ).grid(column=1, row=2, sticky="new", padx=5, pady=5)

        self.x = ctk.CTkEntry(
            self,
            width=40,
        )
        self.x.grid(column=0, row=3, sticky="new", padx=5, pady=5)
        self.x.insert(0, "0")
        self.y = ctk.CTkEntry(
            self,
            width=40,
        )
        self.y.grid(column=1, row=3, sticky="new", padx=5, pady=5)
        self.y.insert(0, "0")

        ctk.CTkLabel(self, text="Dist").grid(
            column=2, row=1, sticky="new", padx=5, pady=5
        )
        self.distance = ctk.CTkEntry(
            self,
            width=40,
        )
        self.distance.grid(column=3, row=1, sticky="new", padx=5, pady=5)
        self.distance.insert(0, "0")

        ctk.CTkLabel(self, text="Dir1").grid(
            column=2, row=2, sticky="new", padx=5, pady=5
        )
        ctk.CTkLabel(
            self,
            text="Dir2",
        ).grid(column=3, row=2, sticky="new", padx=5, pady=5)

        self.dir1 = ctk.CTkEntry(
            self,
            width=40,
        )
        self.dir1.grid(column=2, row=3, sticky="new", padx=5, pady=5)
        self.dir1.insert(0, "0")
        self.dir2 = ctk.CTkEntry(self, width=40)
        self.dir2.grid(column=3, row=3, sticky="new", padx=5, pady=5)
        self.dir2.insert(0, "0")

        self.parent = parent

    def place(self, *args):
        self.parent.distanceBaseline()

    def reset(self, *args):
        self.parent.resetDistance()

    def getDims(self):
        """_summary_

        Returns:
            x and y dimensions for the plane establishing points
        """
        return float(self.x.get()), float(self.y.get())

    def getDisDir(self):
        """_summary_

        Returns:
            directions and distance for goal
        """
        return float(self.distance.get()), int(self.dir1.get()), int(self.dir2.get())

    def ltl(self, *args):
        self.parent.defineTask(2)

    def pte(self, *args):
        self.parent.defineTask(3)


class Done(ctk.CTkFrame):
    """_summary_
    Button widget for managing utils, like initialize, reset, and go. Extends frame
    """

    def __init__(self, parent, root, fg="transparent"):
        """_summary_
                Initializes widget, establishes buttons
        Args:
            parent (_type_): tells widget where to send callbacks
        """
        ctk.CTkFrame.__init__(self, root, fg_color=fg)
        self.not_cam_2 = tk.IntVar()
        self.not_cam_1 = tk.IntVar()

        self.in_uvs = ctk.CTkButton(
            self, text="Init - UVS", command=self.initUVS, width=20
        )
        self.in_uvs.grid(column=1, row=0, sticky="new", padx=5, pady=5)
        self.in_ibvs = ctk.CTkButton(
            self, text="Init - IBVS", command=self.initIBVS, width=20
        )
        self.in_ibvs.grid(column=2, row=0, sticky="new", padx=5, pady=5)

        self.res = ctk.CTkButton(self, text="Reset", command=self.reset, width=20)
        self.res.grid(column=1, row=1, columnspan=2, sticky="new", padx=5, pady=5)

        ctk.CTkButton(self, text="Go ", command=self.go, width=20).grid(
            column=1, row=2, columnspan=2, sticky="new", padx=5, pady=5
        )

        ctk.CTkCheckBox(self, text="Cam 1 Only", variable=self.not_cam_2).grid(
            column=0, row=0, sticky="new", padx=5, pady=5
        )
        ctk.CTkCheckBox(self, text="Cam 2 Only", variable=self.not_cam_1).grid(
            column=0, row=1, sticky="new", padx=5, pady=5
        )

        self.root = root
        self.parent = parent

    def getCamStatus(self):
        return self.not_cam_2.get(), self.not_cam_1.get()

    def initUVS(self, *args):
        self.parent.initTrackers("uvs", self.not_cam_2.get(), self.not_cam_1.get())
        self.root.enableTrackers(True)
        self.root.enableTasks(True)

    def initIBVS(self, *args):
        self.parent.initTrackers("ibvs", self.not_cam_2.get(), self.not_cam_1.get())
        self.root.enableTrackers(True)
        self.root.enableTasks(True)

    def reset(self, *args):
        self.parent.resetAll()
        self.root.enableTrackers(True)
        self.root.enableTasks(True)

    def go(self, *args):
        self.parent.go(self.not_cam_2.get(), self.not_cam_1.get())
        self.root.enableTrackers(False)
        self.root.enableTasks(False)
