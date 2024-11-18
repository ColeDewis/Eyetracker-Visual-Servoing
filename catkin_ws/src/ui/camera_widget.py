import customtkinter as ctk
import rospy
import sensor_msgs.msg as ms
from std_msgs.msg import Bool
import cv2
import numpy as np

from PIL import Image, ImageTk
from cv_bridge import CvBridge
from ws_utils.enums import *

from custom_msgs.msg import (
    TrackComponent,
    ErrorDefinition,
    Point2D,
    DistanceDefinition,
    UVSRequestAction,
    UVSRequestGoal,
)


class CameraDisplays(ctk.CTkFrame):
    """_summary_

    Widget that extends tk.Frame. Handles showing camera images, placing trackers, placing distance definitions, and zooming in.

    """

    def __init__(self, parent, sub, compressed=True):
        """_summary_
            Initializes elements, sets up
        Args:
            parent: lets the widget know where to send information
            sub integer: camera idx for establishing subscribers
        """
        ctk.CTkFrame.__init__(self, parent)
        if compressed:
            self.c = ms.CompressedImage
        else:
            self.c = ms.Image

        self.sub_name = "/compressed" if compressed else ""

        self.image = ctk.CTkLabel(self, text="")
        self.image.grid(column=0, row=1, sticky="ew")
        self.parent = parent
        self.bridge = CvBridge()

        vals = self.parent.getCamIds()

        self.idx_select = ctk.CTkOptionMenu(self, values=vals, command=self.updateFeed)
        self.idx_select.grid(column=0, row=0)

        self.imSub = rospy.Subscriber(
            "/cameras/cam" + str(sub) + self.sub_name, self.c, self.updateIm
        )

        self.image.bind("<ButtonRelease>", None)
        self.paused = False
        self.cnt = 0
        self.placed = 0
        self.mssg = None
        self.points = []

        self.zoom = False
        self.zoomed = False
        self.compass = False

        self.cam_idx = sub
        self.img = None
        self.idx_select.set(str(sub))

    def updateIm(self, data):
        """_summary_
                Callback function for subscriber. Updates image that's displayed to show realtime camera images.

        Args:
            data (ms.Image): image message from camera to be displayed.
        """
        if not self.paused:

            if self.c == ms.CompressedImage:
                frame = self.bridge.compressed_imgmsg_to_cv2(
                    cmprs_img_msg=data, desired_encoding="bgr8"
                )
            else:
                frame = self.bridge.imgmsg_to_cv2(img_msg=data, desired_encoding="rgb8")
            self.size = (len(frame[0]), len(frame))
            if self.compass:
                for i in range(1, len(self.compass_points)):
                    cv2.putText(
                        frame,
                        "d" + str(i - 1),
                        (
                            round(self.compass_points[i][0] - 8),
                            round(self.compass_points[i][1]),
                        ),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (0, 0, 0),
                        thickness=2,
                    )
                    frame = cv2.line(
                        np.array(frame),
                        [
                            round(self.compass_points[0][0]),
                            round(self.compass_points[0][1]),
                        ],
                        [
                            round(self.compass_points[i][0]),
                            round(self.compass_points[i][1]),
                        ],
                        0xFF0000,
                        thickness=1,
                    )
            # frame = cv2.resize(frame, (640, 370))
            im = Image.fromarray(frame)
            imgtk = ctk.CTkImage(im, size=self.size)  # (720, 360))

            self.image.configure(image=imgtk, require_redraw=True)

            self.image.image = imgtk
            self.img = im
            # print(type(self.img))
            self.scale_x = self.image.winfo_width() / 20
            self.scale_y = self.image.winfo_height() / 20

    def calculateCompass(self, m1, m2, l1, l2):
        if l1 > l2:
            scale = l2 / l1
            l1 = self.size[1] / 10
            l2 = (self.size[0] / 10) * scale

        else:
            scale = l1 / l2
            l1 = (self.size[1] / 10) * scale
            l2 = self.size[0] / 10

        center = [(self.size[0] / 20) + 10, (self.size[1] / 20) + 20]

        c1 = 1 / np.sqrt(1 + m1**2)
        s1 = m1 / np.sqrt(1 + m1**2)

        c2 = 1 / np.sqrt(1 + m2**2)
        s2 = m2 / np.sqrt(1 + m2**2)

        p2 = [center[0] + (l1 / 2 * c1), center[1] + (l1 / 2 * s1)]
        p0 = [center[0] - (l1 / 2 * c1), center[1] - (l1 / 2 * s1)]
        p1 = [center[0] + (l2 / 2 * c2), center[1] + (l2 / 2 * s2)]
        p3 = [center[0] - (l2 / 2 * c2), center[1] - (l2 / 2 * s2)]
        self.compass = True
        self.compass_points = [center, p0, p1, p2, p3]

    def pointPlace(
        self, zoomed: bool, cnt: int, mssg, t=None, t_type=None, seg_color=None
    ):
        """_summary_

                Sets up to place a point, sets callback for mouse click, sets up correct types for things, etc etc

        Args:
            zoomed (bool): whether or not the tracker should be zoomed in
            cnt (int): number of points to place
            mssg (_type_): message that the information will be attached to
            t (_type_, optional): tracker type. Defaults to None.
        """
        self.mssg = mssg
        self.placed = 0
        self.cnt = cnt
        self.paused = True

        self.image.bind("<ButtonRelease>", self.click_callback)
        self.zoom = zoomed
        self.t = t
        self.t_type = t_type
        self.points = []
        self.color = seg_color

    def click_callback(self, event):
        """_summary_

                Triggered when there's a click in the image. handles either saving the click location, or calling the zoom function

        Args:
            event (_type_): click location
        """
        if not self.zoom:
            self.placed += 1
            self.drawPoint(event.x, event.y)
            self.points.append([event.x, event.y])

        else:
            if self.zoomed:
                self.placed += 1

                new_x = event.x
                new_y = event.y

                # figure out x, y conversion
                x = new_x / 10 + self.xrange[0]
                y = new_y / 10 + self.yrange[0]

                rospy.loginfo(f"Got click: {x}, {y}")
                self.points.append([x, y])

                self.drawPoint(x, y)
                self.zoomed = False

            else:
                self.zoomIn(event.x, event.y)
                self.zoomed = True
        if self.placed >= self.cnt:
            self.shutdown()

    def zoomIn(self, x, y):
        """_summary_
                Zooms the image in for more accurate placing of the point.
        Args:
            x (_type_): x location of the click
            y (_type_): y location of the click
        """
        if self.scale_x > x:
            self.xrange = [0, round(2 * self.scale_x)]
        elif self.scale_x > self.image.winfo_width() - x:
            self.xrange = [
                round(self.image.winfo_width() - 2 * self.scale_x),
                self.image.winfo_width(),
            ]
        else:
            self.xrange = [round(x - self.scale_x), round(x + self.scale_x)]
        if self.scale_y > y:
            self.yrange = [0, round(2 * self.scale_y)]
        elif self.scale_y > self.image.winfo_height() - y:
            self.yrange = [
                round(self.image.winfo_height() - 2 * self.scale_y),
                self.image.winfo_height(),
            ]
        else:
            self.yrange = [round(y - self.scale_y), round(y + self.scale_y)]

        newim = np.array(self.img)[
            self.yrange[0] : self.yrange[1], self.xrange[0] : self.xrange[1]
        ]
        newim = Image.fromarray(newim)
        newim = newim.resize((self.image.winfo_width(), self.image.winfo_height()))
        newim = ctk.CTkImage(newim, size=self.size)
        self.image.configure(image=newim)
        self.image.image = newim

    def drawPoint(self, x, y):
        """_summary_
                Util function that draws point onto the image, so that the user is aware of where they've already clicked for a given shape. Especially helpful for conics

        Args:
            x (_type_): center of the point, x direction
            y (_type_): center of the point, y direction
        """
        img = cv2.circle(
            np.array(self.img), [round(x), round(y)], 2, 0xFF0000, thickness=-1
        )
        img = Image.fromarray(img)
        self.img = img
        img = ImageTk.PhotoImage(image=img)
        self.image.configure(image=img)
        self.image.image = img

    def tracks(self, i):
        """_summary_
                Util function that switches subscribers from subscribing to the camera to subscribing to the trackers
        Args:
            i (_type_): cam_idx for subscriber
        """
        self.imSub.unregister()
        self.imSub = rospy.Subscriber(
            "/cameras/cam%s/tracked_points" % str(i) + self.sub_name,
            self.c,
            self.updateIm,
        )

    def shutdown(self):
        """_summary_

        Stops the widget from accepting any more clicks
        """
        # self.image.bind("<ButtonRelease>", None)
        self.image.unbind("<ButtonRelease>")
        self.paused = False
        self.placed = 0
        self.cnt = 0
        self.zoom, self.zoomed = False, False
        if type(self.mssg) == ErrorDefinition:
            track = TrackComponent()
            track.type = self.t
            track.track_type = self.t_type
            if self.t_type == TrackerType.SEGMENTATION:
                track.seg_color = self.color
            for i in self.points:
                p = Point2D()
                p.x, p.y = i
                track.points.append(p)

            self.mssg.components.append(track)

        elif type(self.mssg) == DistanceDefinition:
            for i in self.points:
                p = Point2D()
                p.x, p.y = i
                self.mssg.plane_points.append(p)

            self.parent.requestCompass(self.cam_idx)

    def reset(self, sub):
        self.imSub.unregister()
        self.imSub = rospy.Subscriber(
            "/cameras/cam" + str(sub) + self.sub_name, self.c, self.updateIm
        )

        self.image.bind("<ButtonRelease>", None)

        self.paused = False
        self.cnt = 0
        self.placed = 0
        self.mssg = None
        self.points = []

        self.zoom = False
        self.zoomed = False

    def updateFeed(self, new_idx):
        self.idx_select.configure(values=self.parent.getCamIds())
        self.imSub.unregister()
        self.imSub = rospy.Subscriber(
            "/cameras/cam" + new_idx + self.sub_name, self.c, self.updateIm
        )
        print("/cameras/cam" + new_idx + self.sub_name)
