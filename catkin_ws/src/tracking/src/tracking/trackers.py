import cv2
from abc import ABC, abstractmethod
from numpy.typing import ArrayLike
from ws_utils.enums import TrackerType
from sensor_msgs.msg import Image
import cv_bridge
import rospy
import numpy as np


class Tracker(ABC):
    """Abstract Tracker class. Defines methods trackers must implement."""

    def __init__(self, init_frame, init_points):
        """Initialize a tracker.

        Args:
            init_frame (ArrayLike): initial image frame
            init_points (ArrayLike): initially selected points
        """
        pass

    @abstractmethod
    def update_tracking(self, new_frame) -> list:
        """Updates the tracked values, returning the new points.

        Args:
            init_frame (ArrayLike): new image frame

        Returns:
            list: tracked point(s)
        """
        pass


class LKPyrTracker(Tracker):
    """Lucas Kanade pyramid tracker."""

    def __init__(self, init_frame, init_points):
        """Initialize the tracker.

        Args:
            init_frame (ArrayLike): initial image frame
            init_points (ArrayLike): initially selected points
        """
        self.points = init_points
        self.last_frame = cv2.cvtColor(init_frame, cv2.COLOR_BGR2GRAY)

    def update_tracking(self, new_frame) -> list:
        """Updates the tracked values, returning the new points.

        Args:
            init_frame (ArrayLike): new image frame

        Returns:
            list: tracked point(s)
        """
        gray_img = cv2.cvtColor(new_frame, cv2.COLOR_BGR2GRAY)

        lk_params = dict(
            winSize=(32, 32),
            maxLevel=8,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 9, 0.02555),
        )
        new_pts, _, _ = cv2.calcOpticalFlowPyrLK(
            self.last_frame, gray_img, self.points, None, **lk_params
        )
        self.last_frame = gray_img.copy()
        self.points = new_pts

        return new_pts


class FixedTracker(Tracker):
    """Tracks "fixed points" in that they never change."""

    def __init__(self, init_frame, init_points):
        """Initialize the tracker.

        Args:
            init_frame (ArrayLike): initial image frame
            init_points (ArrayLike): initially selected points
        """
        self.points = init_points

    def update_tracking(self, new_frame) -> list:
        """Updates the tracked values, returning the new points.

        Args:
            init_frame (ArrayLike): new image frame

        Returns:
            list: tracked point(s)
        """
        return self.points


class CamShiftTracker(Tracker):
    """Tracks points using camshift.

    See https://docs.opencv.org/3.4/d7/d00/tutorial_meanshift.html
    https://github.com/opencv/opencv/blob/3.4/samples/python/camshift.py
    """

    def __init__(self, init_frame, init_points):
        """Initialize the tracker.

        Args:
            init_frame (ArrayLike): initial image frame
            init_points (ArrayLike): initially selected points
        """
        self.points = init_points
        winsize = 10
        blurred = cv2.medianBlur(init_frame, 17)
        hsv_init = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
        self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)

        self.bound_rects = []
        self.hists = []
        for point in self.points:
            x, y, w, h = (point[0], point[1], winsize, winsize)

            hsv_roi = hsv_init[
                int(y - h / 2) : int(y + h / 2), int(x - w / 2) : int(x + w / 2)
            ]

            # mask low light
            mask_roi = cv2.inRange(
                hsv_roi, np.array((0, 60, 32)), np.array((180, 255, 255))
            )
            hist = cv2.calcHist([hsv_roi], [0], mask_roi, [16], [0, 180])
            cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
            hist = hist.reshape(-1)

            self.hists.append(hist)
            self.bound_rects.append((int(x), int(y), w, h))

    def update_tracking(self, new_frame) -> list:
        """Updates the tracked values, returning the new points.

        Args:
            init_frame (ArrayLike): new image frame

        Returns:
            list: tracked point(s)
        """
        for i, bound_rect in enumerate(self.bound_rects):
            blurred = cv2.medianBlur(new_frame, 17)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
            prob = cv2.calcBackProject([hsv], [0], self.hists[i], [0, 180], 1)

            track_box, new_rect = cv2.CamShift(prob, bound_rect, self.term_crit)

            if track_box[0] != (0.0, 0.0):
                self.bound_rects[i] = new_rect
                self.points[i] = list(map(int, track_box[0]))

        return self.points


class SegmentationCamShiftLayered(Tracker):
    def __init__(
        self,
        init_frame,
        init_points,
        filter_low_mask=(45, 45, 45),
        filter_high_mask=(95, 255, 255),
        debug=False,
    ):
        """Initialize Segmentation-Camshift multi-layered tracking.

        Only works for a single point tracking.

        Args:
            filter_low_mask (list): hsv low mask, defaults to a green mask
            filter_high_mask (list): hsv high mask, defaults to a green mask
        """
        self.filter_low_mask = filter_low_mask
        self.filter_high_mask = filter_high_mask
        self.bound_rect = None
        self.points = init_points
        self.debug = debug

        if self.debug:
            self.debug_pub = rospy.Publisher(
                f"/debug/segment_layered/{np.random.randint(1, 500)}",
                Image,
                queue_size=10,
            )
            self.br = cv_bridge.CvBridge()

    def update_tracking(self, frame):
        """Update Layered Segmentation tracking

        Args:
            frame (list): image frame to update with

        Returns:
            list: new point
        """
        # blur img and convert to HSV for thresholding
        blurred = cv2.medianBlur(frame, 17)
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)  # BGR

        # mask created with HSV thresholds
        mask = cv2.inRange(hsv, self.filter_low_mask, self.filter_high_mask)

        # run opening to try and reduce noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # run closing to try to fill holes
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Mask the blurred image so that we only consider the areas with the desired colour
        masked_blurred = cv2.bitwise_and(blurred, blurred, mask=mask)

        if self.bound_rect is None:
            # find edges and contours
            t_low, t_high = 50, 100
            canny = cv2.Canny(masked_blurred, t_low, t_high)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            dilated = cv2.dilate(canny, kernel)

            contours, hierarchy = cv2.findContours(
                dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )
            # no contours found
            if not contours:
                return self.points

            c = max(contours, key=cv2.contourArea)

            if cv2.contourArea(c) < 100:
                return self.points

            c_poly = cv2.approxPolyDP(c, 3, True)
            self.bound_rect = cv2.boundingRect(c_poly)
            x, y, w, h = self.bound_rect

            cv2.drawContours(frame, [c], -1, (0, 255, 0), 1)
            M = cv2.moments(c)

            if M["m00"] == 0:
                return self.points

            # centroid
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)

            hsv_roi = hsv[
                int(y - h / 2) : int(y + h / 2), int(x - w / 2) : int(x + w / 2)
            ]
            mask_roi = mask[
                int(y - h / 2) : int(y + h / 2), int(x - w / 2) : int(x + w / 2)
            ]
            hist = cv2.calcHist([hsv_roi], [0], mask_roi, [16], [0, 180])
            cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
            self.hist = hist.reshape(-1)

            # if self.debug:
            #     self.debug_pub.publish(self.br.cv2_to_imgmsg(frame, encoding="rgb8"))

            self.points = np.array([[cX, cY]])
            return self.points
        else:
            prob = cv2.calcBackProject([hsv], [0], self.hist, [0, 180], 1)
            prob &= mask
            term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
            track_box, self.bound_rect = cv2.CamShift(prob, self.bound_rect, term_crit)
            if track_box[0] == (0.0, 0.0):
                self.bound_rect = None

            if self.debug:
                box = cv2.drawContours(
                    frame.copy(), [np.intp(cv2.boxPoints(track_box))], 0, (0, 255, 0)
                )
                self.debug_pub.publish(
                    self.br.cv2_to_imgmsg(masked_blurred, encoding="rgb8")
                )

            self.points = np.array([list(map(float, track_box[0]))])
            return self.points


class SegmentationTracker(Tracker):
    """Tracks points using camshift."""

    def __init__(self, init_frame, init_points, seg_color):
        """Initialize the tracker.

        Args:
            init_frame (ArrayLike): initial image frame
            init_points (ArrayLike): initially selected points - ((low,low,low),(high,high,high))
        """
        """hsv = cv2.cvtColor(init_frame, cv2.COLOR_BGR2HSV)
        init_region = hsv[
            round(init_points[0]) - 5 : round(init_points[0]) + 5,
            round(init_points[1]) - 5 : round(init_points[1]) + 5,
        ]

        average = init_region.mean(axis=0).mean(axis=0)

        self.low_filter, self.high_filter = (0.6 * average[0], 30, 30), (
            min(3 * average[0], 180),
            255,
            255,
        )"""

        self.low_filter, self.high_filter = seg_color[0:3], seg_color[3:6]
        self.last_frame = init_frame
        self.t_low, self.t_high = 50, 100
        self.points = init_points

    def update_tracking(self, new_frame) -> list:
        """Updates the tracked values, returning the new points.

        Args:
            init_frame (ArrayLike): new image frame

        Returns:
            list: tracked point(s)
        """

        blurred = cv2.medianBlur(new_frame, 17)
        hsv = cv2.cvtColor(new_frame, cv2.COLOR_BGR2HSV)

        # mask created with HSV thresholds
        mask = cv2.inRange(hsv, self.low_filter, self.high_filter)

        # run opening to try and reduce noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # run closing to try to fill holes
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Mask the blurred image so that we only consider the areas with the desired colour
        masked_blurred = cv2.bitwise_and(blurred, blurred, mask=mask)

        canny = cv2.Canny(masked_blurred, self.t_low, self.t_high)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        dilated = cv2.dilate(canny, kernel)

        contours, hierarchy = cv2.findContours(
            dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        # no contours found
        if not contours:
            return self.points

        c = max(contours, key=cv2.contourArea)

        if cv2.contourArea(c) < 100:
            return self.points

        c_poly = cv2.approxPolyDP(c, 3, True)
        self.bound_rect = cv2.boundingRect(c_poly)
        x, y, w, h = self.bound_rect

        # cv2.drawContours(new_frame, [c], -1, (0, 255, 0), 1)
        M = cv2.moments(c)

        if M["m00"] == 0:
            return self.points

        # centroid
        cX = M["m10"] / M["m00"]
        cY = M["m01"] / M["m00"]

        self.points = np.array([[cX, cY]])

        return self.points
