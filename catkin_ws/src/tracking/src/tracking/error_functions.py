import rospy
import numpy as np
import numpy.typing as npt
from abc import ABC, abstractmethod
import cv2
from random import randint
import sympy as sy

from custom_msgs.msg import ErrorTrackedPoints, TrackComponent, Point2D
from tracking.trackers import Tracker
from tracking.tracker_factory import tracker_factory
from ws_utils.enums import TrackComponentType, ErrorDefinitionType
from ws_utils.conic_util import fit_ellipse


class ErrorFunction(ABC):
    """Abstract ErrorFunction class. Contains logic for updating tracking and calculating error."""

    def __init__(
        self,
        id,
        initial_frame,
        source_points,
        target_points,
        source_track_type,
        target_track_type,
        task_scale=1.0,
        seg_color=None,
    ):
        """Base constructor for error. Sets up tracking requirements and gets a color for annotation.

        Args:
            initial_frame (list): opencv initial image
            initial_points (np.Array): np array of initial points.
        """
        self.id = id
        self._points = np.vstack([source_points, target_points])
        self.task_scale = task_scale
        self.source_track_type = source_track_type
        self.target_track_type = target_track_type

        # initialize tracking for both components
        self.target_tracker: Tracker = tracker_factory(
            target_track_type, initial_frame, target_points, seg_color=seg_color
        )
        self.source_tracker: Tracker = tracker_factory(
            source_track_type, initial_frame, source_points, seg_color=seg_color
        )

        # random color
        b = randint(150, 255)
        g = randint(150, 255)
        r = randint(150, 255)
        self.color = (b, g, r)

    def get_id(self) -> str:
        """Get the ID for the function.

        Returns:
            str: string id
        """
        return self.id

    def get_error(self) -> list:
        """Get the error value. Can be overridden if custom logic is required. Otherwise, will use self.error_equation and
        self.points to determine the error with sympy.

        Returns:
            list: list containing the error. Since we could have both scalar or vector errors, we return
                them all as vector so that they can all be treated the same way in code by iteration.
        """
        args = {}
        for i, pt in enumerate(self.points):
            args["x" + str(i + 1)] = pt[0]
            args["y" + str(i + 1)] = pt[1]

        err = self.error_equation.evalf(subs=args)

        # duck typing to convert to a list for convenience
        try:
            err = list(err)
        except TypeError:
            err = [err]

        err = np.multiply(err, self.task_scale)

        return err

    def update_tracking(self, new_frame: npt.ArrayLike):
        """Update trackers for the given new_frame. Uses self.points

        Args:
            new_frame (npt.ArrayLike): new camera frame to update tracking for
        """
        # TODO: maybe more explicit to have separate variables?
        target_pts = self.target_tracker.update_tracking(new_frame)
        source_pts = self.source_tracker.update_tracking(new_frame)
        self.points = np.vstack([source_pts, target_pts])

    @property
    def points(self):
        """Property holding the points for the error."""
        return self._points

    @points.setter
    def points(self, new_pts):
        """Setter for points property

        Args:
            new_pts (list): new points
        """
        self._points = new_pts

    @property
    @abstractmethod
    def error_equation(self):
        """Property holding the error equation."""
        pass

    @abstractmethod
    def draw(self, image, thickness):
        """Draw the target on an image. Should be implemented by subclass.

        Args:
            image (list): image array
            thickness (int): thickness to draw
        """
        pass

    @abstractmethod
    def get_msg(self) -> ErrorTrackedPoints:
        """Get the ROS message for the error function. Should be implemented by subclass

        Returns:
            ErrorTrackedPoints: error message containing info about the tracked points
        """
        pass


class PointToPoint(ErrorFunction):
    """Point to point constraint class. Implements ErrorFunction"""

    def __init__(
        self,
        id,
        initial_frame,
        source_pt,
        target_pt,
        source_track_type,
        target_track_type,
        task_scale=1.0,
        dist_ref_pts=None,
        seg_color=None,
    ):
        """Create a point_to_point error.

        Args:
            initial_frame (list): opencv initial image
            initial_points (np.Array): np array of initial points.
            static_indexes (list): list of point indexes that should NOT be updated by tracking.
        """
        super().__init__(
            id,
            initial_frame,
            source_pt,
            target_pt,
            source_track_type,
            target_track_type,
            task_scale,
            seg_color=seg_color,
        )

        self.dist_ref_pts = dist_ref_pts
        x1, y1, x2, y2 = sy.symbols("x1 y1 x2 y2")
        xy1 = sy.Matrix([x1, y1])
        xy2 = sy.Matrix([x2, y2])
        self._error_equation = xy1 - xy2

    @property
    def error_equation(self):
        """Property holding the error equation."""
        return self._error_equation

    def draw(self, image, thickness):
        """Draw the error on an image.

        Args:
            image (list): image array
            thickness (int): thickness to draw
        """
        p1 = list(map(int, self.points[0]))
        p2 = list(map(int, self.points[1]))
        cv2.circle(image, p1, 0, color=self.color, thickness=thickness)
        cv2.circle(image, p2, 0, color=self.color, thickness=thickness)

        # TODO: refactor to superclass?
        if self.dist_ref_pts is not None:
            for pt in self.dist_ref_pts:
                cv2.circle(image, (int(pt[0]), int(pt[1])), 5, (255, 0, 0), 3)

    def get_msg(self) -> ErrorTrackedPoints:
        """Get the ROS message for the error function.

        Returns:
            ErrorTrackedPoints: error message containing info about the tracked points
        """
        msg = ErrorTrackedPoints()
        p1 = list(map(int, self.points[0]))
        p2 = list(map(int, self.points[1]))

        msg.header.frame_id = ""
        msg.header.stamp = rospy.get_rostime()
        msg.type = ErrorDefinitionType.POINT_POINT
        msg.id = self.id
        pt1_component = TrackComponent()
        pt1_component.type = TrackComponentType.ANY_POINT
        pt1_component.track_type = self.source_track_type
        pt1_component.points = [Point2D(x=p1[0], y=p1[1])]
        pt2_component = TrackComponent()
        pt2_component.type = TrackComponentType.ANY_POINT
        pt2_component.track_type = self.target_track_type
        pt2_component.points = [Point2D(x=p2[0], y=p2[1])]
        msg.components = [pt1_component, pt2_component]

        return msg


class PointToLine(ErrorFunction):
    """Point to line constraint class. Implements ErrorFunction"""

    def __init__(
        self,
        id,
        initial_frame,
        source_point,
        target_line,
        source_track_type,
        target_track_type,
        task_scale=1.0,
        dist_ref_pts=None,
    ):
        """Create a point to line error. It is assumed that initial_points contains first the point, followed by the two points for the line.

        Args:
            initial_frame (list): opencv initial image
            initial_points (np.Array): np array of initial points.
            static_indexes (list): list of point indexes that should NOT be updated by tracking.
        """
        rospy.loginfo(f"pt: {source_point}, ln {target_line}")
        super().__init__(
            id,
            initial_frame,
            source_point,
            target_line,
            source_track_type,
            target_track_type,
            task_scale,
        )
        self.dist_ref_pts = dist_ref_pts

        x1, y1, x2, y2, x3, y3 = sy.symbols("x1 y1 x2 y2 x3 y3")
        xy1 = sy.Matrix([x1, y1, 1])
        xy2 = sy.Matrix([x2, y2, 1])
        xy3 = sy.Matrix([x3, y3, 1])
        self._error_equation = xy1.dot(xy2.cross(xy3))

    @property
    def error_equation(self):
        """Property holding the error equation."""
        return self._error_equation

    def draw(self, image, thickness):
        """Draw the error on an image.

        Args:
            image (list): image array
            thickness (int): thickness to draw
        """
        p1 = list(map(int, self.points[0]))
        p2 = list(map(int, self.points[1]))
        p3 = list(map(int, self.points[2]))

        cv2.circle(image, p1, 0, color=self.color, thickness=thickness)
        cv2.line(image, p2, p3, color=self.color, thickness=thickness)

        if self.dist_ref_pts is not None:
            for pt in self.dist_ref_pts:
                cv2.circle(image, (int(pt[0]), int(pt[1])), 5, (255, 0, 0), 3)

    def get_msg(self) -> ErrorTrackedPoints:
        """Get the ROS message for the error function.

        Returns:
            ErrorTrackedPoints: error message containing info about the tracked points
        """
        msg = ErrorTrackedPoints()
        p1 = list(map(int, self.points[0]))
        p2 = list(map(int, self.points[1]))
        p3 = list(map(int, self.points[2]))

        msg.header.frame_id = ""
        msg.header.stamp = rospy.get_rostime()
        msg.type = ErrorDefinitionType.POINT_LINE
        msg.id = self.id
        pt1_component = TrackComponent()
        pt1_component.type = TrackComponentType.ANY_POINT
        pt1_component.track_type = self.source_track_type
        pt1_component.points = [Point2D(x=p1[0], y=p1[1])]
        ln1_component = TrackComponent()
        ln1_component.type = TrackComponentType.ANY_LINE
        ln1_component.track_type = self.target_track_type
        ln1_component.points = [Point2D(x=p2[0], y=p2[1]), Point2D(x=p3[0], y=p3[1])]
        msg.components = [pt1_component, ln1_component]

        return msg


class LineToLine(ErrorFunction):
    """Line to Line constraint class. Implements ErrorFunction"""

    def __init__(
        self,
        id,
        initial_frame,
        source_line,
        target_line,
        source_track_type,
        target_track_type,
        task_scale=1.0,
        dist_ref_pts=None,
    ):
        """Create a line to line error. It is assumed that initial_points contains first the first line, then the second.

        Args:
            initial_frame (list): opencv initial image
            initial_points (np.Array): np array of initial points.
            static_indexes (list): list of point indexes that should NOT be updated by tracking.
        """
        super().__init__(
            id,
            initial_frame,
            source_line,
            target_line,
            source_track_type,
            target_track_type,
            task_scale,
        )
        self.dist_ref_pts = dist_ref_pts

        x1, y1, x2, y2, x3, y3, x4, y4 = sy.symbols("x1 y1 x2 y2 x3 y3 x4 y4")
        xy1 = sy.Matrix([x1, y1, 1])
        xy2 = sy.Matrix([x2, y2, 1])
        xy3 = sy.Matrix([x3, y3, 1])
        xy4 = sy.Matrix([x4, y4, 1])
        l2 = xy3.cross(xy4)
        self._error_equation = xy1.dot(l2) + xy2.dot(l2)

    @property
    def error_equation(self):
        """Property holding the error equation."""
        return self._error_equation

    def draw(self, image, thickness):
        """Draw the error on an image.

        Args:
            image (list): image array
            thickness (int): thickness to draw
        """
        p1 = list(map(int, self.points[0]))
        p2 = list(map(int, self.points[1]))
        p3 = list(map(int, self.points[2]))
        p4 = list(map(int, self.points[3]))

        cv2.line(image, p1, p2, color=self.color, thickness=thickness)
        cv2.line(image, p3, p4, color=self.color, thickness=thickness)

        if self.dist_ref_pts is not None:
            for pt in self.dist_ref_pts:
                cv2.circle(image, (int(pt[0]), int(pt[1])), 5, (255, 0, 0), 3)

    def get_msg(self) -> ErrorTrackedPoints:
        """Get the ROS message for the error function.

        Returns:
            ErrorTrackedPoints: error message containing info about the tracked points
        """
        msg = ErrorTrackedPoints()
        p1 = list(map(int, self.points[0]))
        p2 = list(map(int, self.points[1]))
        p3 = list(map(int, self.points[2]))
        p4 = list(map(int, self.points[3]))

        msg.header.frame_id = ""
        msg.header.stamp = rospy.get_rostime()
        msg.type = ErrorDefinitionType.LINE_LINE
        msg.id = self.id
        ln1_component = TrackComponent()
        ln1_component.type = TrackComponentType.ANY_LINE
        ln1_component.track_type = self.source_track_type
        ln1_component.points = [Point2D(x=p1[0], y=p1[1]), Point2D(x=p2[0], y=p2[1])]
        ln2_component = TrackComponent()
        ln2_component.type = TrackComponentType.ANY_LINE
        ln2_component.track_type = self.target_track_type
        ln2_component.points = [Point2D(x=p3[0], y=p3[1]), Point2D(x=p4[0], y=p4[1])]
        msg.components = [ln1_component, ln2_component]

        return msg


class PointToEllipse(ErrorFunction):
    """PointToEllipse constraint class. Implements ErrorFunction"""

    def __init__(
        self,
        id,
        initial_frame,
        source_point,
        target_ellipse,
        source_track_type,
        target_track_type,
        task_scale=1.0,
        dist_ref_pts=None,
    ):
        """Create a point to ellipse (conic) error. It is assumed that initial_points contains first the point, then the 5 conic points.

        Args:
            initial_frame (list): opencv initial image
            initial_points (np.Array): np array of initial points.
            static_indexes (list): list of point indexes that should NOT be updated by tracking.
        """
        super().__init__(
            id,
            initial_frame,
            source_point,
            target_ellipse,
            source_track_type,
            target_track_type,
            task_scale,
        )

        self.dist_ref_pts = dist_ref_pts

        x1, y1, a, b, c, d, e, f = sy.symbols("x1 y1 a b c d e f")
        xy1 = sy.Matrix([x1, y1, 1])
        conic_mat = sy.Matrix([[a, b / 2, d / 2], [b / 2, c, e / 2], [d / 2, e / 2, f]])
        self._error_equation = xy1.T * conic_mat * xy1
        # self._error_equation = 2 * conic_mat * xy1

        deriv_mat = sy.Matrix(
            [
                [2 * a * x1 + b * y1 + c * y1**2 + d + e * y1 + f],
                [a * x1**2 + b * x1 + 2 * c * y1 + d * x1 + e + f],
            ]
        )
        # self._error_equation = deriv_mat

        self.conic_coeffs = None

    @property
    def error_equation(self):
        """Property holding the error equation."""
        return self._error_equation

    def get_error(self) -> list:
        """Get the error value. This overrides the base class, since here we need extra logic using conic solving and SVD.

        Returns:
            list: list containing the error.
        """

        pts = self.points[1:]
        conic = fit_ellipse(pts)[0]

        args = {}
        args["x1"] = self.points[0][0]
        args["y1"] = self.points[0][1]
        args["a"] = conic[0]
        args["b"] = conic[1]
        args["c"] = conic[2]
        args["d"] = conic[3]
        args["e"] = conic[4]
        args["f"] = conic[5]

        self.conic_coeffs = conic

        err = self.error_equation.evalf(subs=args)
        err = np.multiply(err, self.task_scale)

        # return err
        return [err[0]]

    def draw(self, image, thickness):
        """Draw the error on an image.

        Args:
            image (list): image array
            thickness (int): thickness to draw
        """
        # Math used here is from: https://math.stackexchange.com/a/820896
        if self.conic_coeffs is None:
            return

        A, B, C, D, E, F = self.conic_coeffs

        if B**2 - 4 * A * C >= 0:
            # in the future handle this by just not drawing one that frame
            raise TypeError("couldn't fit ellipse")

        q = (
            64
            * (F * (4 * A * C - B**2) - A * E**2 + B * D * E - C * D**2)
            / (4 * A * C - B**2) ** 2
        )
        s = 1 / 4 * np.sqrt(np.abs(q) * np.sqrt(B**2 + (A - C) ** 2))

        rmax = (
            1
            / 8
            * np.sqrt(2 * np.abs(q) * np.sqrt(B**2 + (A - C) ** 2) - 2 * q * (A + C))
        )
        rmin = np.sqrt(rmax**2 - s**2)
        x_center = (B * E - 2 * C * D) / (4 * A * C - B**2)
        y_center = (B * D - 2 * A * E) / (4 * A * C - B**2)

        qa_minus_qc = q * A - q * C
        qb = q * B
        if qa_minus_qc == 0 and qb == 0:
            angle = 0
        elif qa_minus_qc == 0 and qb > 0:
            angle = np.pi / 4
        elif qa_minus_qc == 0 and qb < 0:
            angle = 3 * np.pi / 4
        elif qa_minus_qc > 0 and qb >= 0:
            angle = 1 / 2 * np.arctan(B / (A - C))
        elif qa_minus_qc > 0 and qb < 0:
            angle = 1 / 2 * np.arctan(B / (A - C)) + np.pi
        elif qa_minus_qc < 0:
            angle = 1 / 2 * np.arctan(B / (A - C)) + np.pi / 2

        axes = (int(rmax), int(rmin))
        center = (int(x_center), int(y_center))
        cv2.ellipse(
            image,
            center,
            axes,
            np.rad2deg(angle),
            0,
            360,
            color=self.color,
            thickness=thickness,
        )
        cv2.circle(
            image,
            (int(self.points[0][0]), int(self.points[0][1])),
            0,
            color=self.color,
            thickness=thickness,
        )

    def get_msg(self) -> ErrorTrackedPoints:
        """Get the ROS message for the error function.

        Returns:
            ErrorTrackedPoints: error message containing info about the tracked points
        """
        msg = ErrorTrackedPoints()
        p1 = list(map(int, self.points[0]))
        conic_pts = self.points[1:]

        msg.header.frame_id = ""
        msg.header.stamp = rospy.get_rostime()
        msg.type = ErrorDefinitionType.POINT_CONIC
        msg.id = self.id
        pt1_component = TrackComponent()
        pt1_component.type = TrackComponentType.ANY_POINT
        pt1_component.track_type = self.source_track_type
        pt1_component.points = [Point2D(x=p1[0], y=p1[1])]
        cn1_component = TrackComponent()
        cn1_component.type = TrackComponentType.CONIC
        cn1_component.track_type = self.target_track_type
        cn1_component.points = [Point2D(x=int(pt[0]), y=int(pt[1])) for pt in conic_pts]
        msg.components = [pt1_component, cn1_component]

        return msg


class Waypoints(ErrorFunction):
    """Error function that chains multiple point-point errors together. Implements ErrorFunction."""

    def __init__(
        self,
        id,
        initial_frame,
        source_point,
        target_waypoints,
        source_track_type,
        waypoint_track_type,
        ibvs: bool,
        task_scale: float = 1.0,
    ):
        """Create a waypoint error.

        Args:
            initial_frame (list): opencv initial image
            initial_points (np.Array): np array of initial points.
            static_indexes (list): list of point indexes that should NOT be updated by tracking.
        """
        super().__init__(
            id,
            initial_frame,
            source_point,
            target_waypoints,
            source_track_type,
            waypoint_track_type,
            task_scale=task_scale,
        )

        self.target_waypoints = target_waypoints

        x1, y1, x2, y2 = sy.symbols("x1 y1 x2 y2")
        if ibvs:
            xy1 = sy.Matrix([x1, y1, x1, y1, x1, y1, x1, y1])
            xy2 = sy.Matrix([x2, y2, x2, y2, x2, y2, x2, y2])
        else:
            xy1 = sy.Matrix([x1, y1])
            xy2 = sy.Matrix([x2, y2])
        self._error_equation = xy2 - xy1
        self._current_waypoint_idx = 0

    @property
    def error_equation(self):
        """Property holding the error equation."""
        return self._error_equation

    def get_error(self) -> list:
        """Get the error value. Between the tracked point and current waypoint

        Returns:
            list: list containing the error. Since we could have both scalar or vector errors, we return
                them all as vector so that they can all be treated the same way in code by iteration.
        """
        args = {}
        args["x1"] = self.points[0][0]
        args["y1"] = self.points[0][1]
        args["x2"] = self.points[self._current_waypoint_idx + 1][0]
        args["y2"] = self.points[self._current_waypoint_idx + 1][1]

        err = self.error_equation.evalf(subs=args)
        err = np.multiply(list(err), self.task_scale)

        return err

    def swap_to_next_waypoint(self) -> bool:
        """Swap the waypoint target to the next waypoint.

        Returns:
            bool: true if successful, false if there is not another waypoint to swap to
        """
        if self._current_waypoint_idx == len(self.target_waypoints) - 1:
            return False
        else:
            self._current_waypoint_idx += 1
            return True

    def draw(self, image, thickness):
        """Draw the error on an image.

        Args:
            image (list): image array
            thickness (int): thickness to draw
        """
        for i, point in enumerate(self.points):
            pt = list(map(int, point))
            if i in (self._current_waypoint_idx, len(self.points) - 1):
                cv2.circle(image, pt, 0, color=self.color, thickness=thickness)
            else:
                cv2.circle(image, pt, 5, color=self.color, thickness=3)

    def get_msg(self) -> ErrorTrackedPoints:
        """Get the ROS message for the error function.

        Returns:
            ErrorTrackedPoints: error message containing info about the tracked points
        """
        # TODO: improve this logic for waypoints specifically?
        msg = ErrorTrackedPoints()
        p1 = list(map(int, self.points[-1]))
        p2 = list(map(int, self.points[self._current_waypoint_idx]))

        msg.header.frame_id = ""
        msg.header.stamp = rospy.get_rostime()
        msg.type = ErrorDefinitionType.WAYPOINTS
        msg.id = self.id
        pt1_component = TrackComponent()
        pt1_component.type = TrackComponentType.ANY_POINT
        pt1_component.track_type = self.source_track_type
        pt1_component.points = [Point2D(x=p1[0], y=p1[1])]
        pt2_component = TrackComponent()
        pt2_component.type = TrackComponentType.WAYPOINTS
        pt2_component.track_type = self.target_track_type
        pt2_component.points = [Point2D(x=p2[0], y=p2[1])]
        msg.components = [pt1_component, pt2_component]

        return msg


class IBVSPointToPoint(ErrorFunction):
    """IBVS Point to Point - tracks one point, but generates a box around the point to use for the error,
    so that we have sufficient information to give the interaction matrix full rank."""

    def __init__(
        self,
        id,
        initial_frame,
        initial_point,
        target_feature,
        source_track_type,
        target_track_type,
        window_size=5,
        task_scale=1.0,
    ):
        """Create a point_to_point error.

        Args:
            initial_frame (list): opencv initial image
            initial_points (np.Array): np array of initial points.
            static_indexes (list): list of point indexes that should NOT be updated by tracking.
        """
        super().__init__(
            id,
            initial_frame,
            initial_point,
            target_feature,
            source_track_type,
            target_track_type,
            task_scale,
        )

        x1, y1, x2, y2 = sy.symbols("x1 y1 x2 y2")
        xy1 = sy.Matrix([x1, y1, 1])
        xy2 = sy.Matrix([x2, y2, 1])
        self._error_equation = (xy1 - xy2)[:2, :]
        self._window_size = window_size

    def get_error(self) -> list:
        """Get the error value.

        Returns:
            list: list containing the error. Since we could have both scalar or vector errors, we return
                them all as vector so that they can all be treated the same way in code by iteration.
        """
        track_point = self.points[0]
        target_point = self.points[1]

        target_box_one = np.array(
            [target_point[0] + self._window_size, target_point[1] + self._window_size]
        )
        target_box_two = np.array(
            [target_point[0] - self._window_size, target_point[1] + self._window_size]
        )
        target_box_three = np.array(
            [target_point[0] + self._window_size, target_point[1] - self._window_size]
        )
        target_box_four = np.array(
            [target_point[0] - self._window_size, target_point[1] - self._window_size]
        )
        track_box_one = np.array(
            [track_point[0] + self._window_size, track_point[1] + self._window_size]
        )
        track_box_two = np.array(
            [track_point[0] - self._window_size, track_point[1] + self._window_size]
        )
        track_box_three = np.array(
            [track_point[0] + self._window_size, track_point[1] - self._window_size]
        )
        track_box_four = np.array(
            [track_point[0] - self._window_size, track_point[1] - self._window_size]
        )

        err = np.array(
            [
                -np.subtract(track_box_one, target_box_one),
                -np.subtract(track_box_two, target_box_two),
                -np.subtract(track_box_three, target_box_three),
                -np.subtract(track_box_four, target_box_four),
            ]
        ).flatten()

        err = np.multiply(err, self.task_scale)

        return err

    @property
    def error_equation(self):
        """Property holding the error equation."""
        return self._error_equation

    def draw(self, image, thickness):
        """Draw the error on an image.

        Args:
            image (list): image array
            thickness (int): thickness to draw
        """
        target_point = self.points[0]
        track_point = self.points[1]

        pts = [
            np.array(
                [
                    target_point[0] + self._window_size,
                    target_point[1] + self._window_size,
                ],
                dtype=int,
            ),
            np.array(
                [
                    target_point[0] - self._window_size,
                    target_point[1] + self._window_size,
                ],
                dtype=int,
            ),
            np.array(
                [
                    target_point[0] + self._window_size,
                    target_point[1] - self._window_size,
                ],
                dtype=int,
            ),
            np.array(
                [
                    target_point[0] - self._window_size,
                    target_point[1] - self._window_size,
                ],
                dtype=int,
            ),
            np.array(
                [
                    track_point[0] + self._window_size,
                    track_point[1] + self._window_size,
                ],
                dtype=int,
            ),
            np.array(
                [
                    track_point[0] - self._window_size,
                    track_point[1] + self._window_size,
                ],
                dtype=int,
            ),
            np.array(
                [
                    track_point[0] + self._window_size,
                    track_point[1] - self._window_size,
                ],
                dtype=int,
            ),
            np.array(
                [
                    track_point[0] - self._window_size,
                    track_point[1] - self._window_size,
                ],
                dtype=int,
            ),
        ]

        cv2.circle(
            image,
            list(map(int, target_point)),
            0,
            color=self.color,
            thickness=thickness,
        )
        cv2.circle(
            image, list(map(int, track_point)), 0, color=self.color, thickness=thickness
        )

        for pt in pts:
            cv2.circle(image, pt, 0, color=self.color, thickness=3)

    def get_msg(self) -> ErrorTrackedPoints:
        """Get the ROS message for the error function.

        Returns:
            ErrorTrackedPoints: error message containing info about the tracked points
        """
        # TODO: what makes sense to fill in here?
        msg = ErrorTrackedPoints()
        p1 = list(map(int, self.points[0]))
        p2 = list(map(int, self.points[1]))

        msg.header.frame_id = ""
        msg.header.stamp = rospy.get_rostime()
        msg.type = ErrorDefinitionType.IBVS_PT_PT
        msg.id = self.id
        pt1_component = TrackComponent()
        pt1_component.type = TrackComponentType.ANY_POINT
        pt1_component.track_type = self.source_track_type
        pt1_component.points = [Point2D(x=p1[0], y=p1[1])]
        pt2_component = TrackComponent()
        pt2_component.type = TrackComponentType.ANY_POINT
        pt2_component.track_type = self.target_track_type
        pt2_component.points = [Point2D(x=p2[0], y=p2[1])]
        msg.components = [pt1_component, pt2_component]

        return msg
