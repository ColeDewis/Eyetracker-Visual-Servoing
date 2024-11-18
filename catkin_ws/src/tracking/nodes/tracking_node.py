import sys
from tracking.tracker_factory import tracker_factory
import rospy
import cv2
import cv_bridge
import numpy as np

from std_msgs.msg import Empty, String
from sensor_msgs.msg import Image, CompressedImage
from visual_servoing.msg import Error, TrackedPoint, TrackedPoints
from custom_msgs.msg import (
    ErrorDefinition,
    Point2D,
    ErrorTrackedPoints,
    CartesianErrorDefinition,
    TrackRequest,
    IDPoint,
)
from custom_msgs.srv import SwapWaypoint, SwapWaypointRequest, SwapWaypointResponse
from tracking.error_functions import (
    ErrorFunction,
)

from tracking.cartesian_error_functions import (
    KinovaJointAngleConstriant,
    KinovaPoseConstriant,
)
from ws_utils.enums import (
    CartesianErrorDefinitionType,
)
from tracking.error_function_factory import error_function_factory


class TrackingNode:
    """Basic Node that runs a camera on a given camera index and publishes to ros."""

    def __init__(self):
        """Initialize a camera node by getting the rosparam and opening the opencv stream."""
        rospy.init_node("tracking_node")
        index_list_param = rospy.search_param("idxs")
        self.cam_indices = rospy.get_param(index_list_param, None)

        if self.cam_indices is None:
            rospy.logwarn('Must pass camera indexes as _idxs:="[idx1, idx2, ...]"')
            exit()

        rospy.delete_param(
            index_list_param
        )  # delete param so its needed for future runs.
        rospy.loginfo(
            f"Initialized TrackingNode with camera indexes {self.cam_indices}"
        )

        self.ready = False
        self.tracking_publishers = {}
        self.compressed_tracking_publishers = {}
        self.camera_subscribers = {}
        self.error_formulas = {}
        self.non_error_trackers = {}
        self.cartesian_error_formulas = []
        self.error_function_publishers = {}
        self.track_point_publishers = {}

        for idx in self.cam_indices:
            self.camera_subscribers[idx] = rospy.Subscriber(
                f"/cameras/cam{idx}",
                Image,
                self.update_trackers,
                queue_size=1,
            )

            self.tracking_publishers[idx] = rospy.Publisher(
                f"/cameras/cam{idx}/tracked_points",
                Image,
                queue_size=10,
            )
            self.compressed_tracking_publishers[idx] = rospy.Publisher(
                f"/cameras/cam{idx}/tracked_points/compressed",
                CompressedImage,
                queue_size=10,
            )
            self.error_function_publishers[idx] = rospy.Publisher(
                f"/tracking_node/cam{idx}/error_functions",
                ErrorTrackedPoints,
                queue_size=10,
            )
            self.track_point_publishers[idx] = rospy.Publisher(
                f"/tracking_node/cam{idx}/tracking", IDPoint, queue_size=10
            )
            self.error_formulas[idx] = []
            self.non_error_trackers[idx] = []

            rospy.loginfo(f"Waiting for message on /cameras/cam{idx}")
            rospy.wait_for_message(f"/cameras/cam{idx}", Image)

        self.bridge = cv_bridge.CvBridge()

        # --- Publishers ---
        self.eef_pos_pub = rospy.Publisher("/eef_pos", TrackedPoints, queue_size=10)

        self.image_error_pub = rospy.Publisher("/image_error", Error, queue_size=10)

        # timer for tracking + error update message publishing
        rospy.Timer(rospy.Duration(0.05), self.publish_tracking_and_error)

        # --- Subscribers ---
        self.error_request_subscriber = rospy.Subscriber(
            "/tracking_node/error_request", ErrorDefinition, self.error_request_callback
        )

        self.non_error_track_req_subscriber = rospy.Subscriber(
            "/tracking_node/track_point_request",
            TrackRequest,
            self.track_request_callback,
        )

        self.cartesian_error_request_subscriber = rospy.Subscriber(
            "/tracking_node/cartesian_error_request",
            CartesianErrorDefinition,
            self.cartesian_error_request_cb,
        )

        self.remove_all_tracking_subscriber = rospy.Subscriber(
            "/tracking_node/reset_all", Empty, self.reset_all_tracking
        )

        self.remove_track_id_subscriber = rospy.Subscriber(
            "/tracking_node/reset_id", String, self.remove_tracking_by_id
        )

        # --- Services ---
        self.swap_waypoint_srv = rospy.Service(
            "/tracking_node/swap_waypoint", SwapWaypoint, self.swap_waypoint_by_id
        )

        rospy.loginfo("Tracking Node is ready!")
        self.ready = True

    def error_request_callback(self, data: ErrorDefinition):
        """Callback when we receive a new error request message

        Args:
            data (ErrorDefinition): error definition to implement
        """
        rospy.loginfo(f"{data}")
        rospy.loginfo("Hit Callback")
        cam_idx = data.cam_idx
        rospy.loginfo(f"Error Request Init - {cam_idx}")

        dist_valid = (
            len(data.distance_info.plane_points) == 4
            and data.distance_info.desired_distance != 0
        )

        starting_frame = rospy.wait_for_message(f"/cameras/cam{cam_idx}", Image)

        starting_frame = self.bridge.imgmsg_to_cv2(
            img_msg=starting_frame, desired_encoding="rgb8"
        )

        rospy.loginfo(f"{np.shape(starting_frame)}")

        func = error_function_factory(data, starting_frame, dist_valid)
        self.error_formulas[cam_idx].append(func)

    def cartesian_error_request_cb(self, data: CartesianErrorDefinition):
        """Error request for cartesian errors (e.g. robot joint/position constraints)

        Args:
            data (CartesianErrorDefinition): error definition
        """
        rospy.loginfo(f"Cartesian error request received: {data.type}")

        if data.type == CartesianErrorDefinitionType.JOINT_ANGLE:
            self.cartesian_error_formulas.append(
                KinovaJointAngleConstriant(
                    data.id, data.component_num, data.desired_value, data.task_scale
                )
            )
        elif data.type == CartesianErrorDefinitionType.POSE:
            self.cartesian_error_formulas.append(
                KinovaPoseConstriant(
                    data.id, data.component_num, data.desired_value, data.task_scale
                )
            )

    def track_request_callback(self, data: TrackRequest):
        """Callback for a request to track a point, but not use it in any error function.

        Args:
            data (TrackRequest): track request message.
        """
        rospy.loginfo(f"Track Request Callback, data: {data}")
        img = rospy.wait_for_message(f"/cameras/cam{data.cam_idx}", Image)
        # if self.compression:
        #     img = self.bridge.compressed_imgmsg_to_cv2(img)

        img = self.bridge.imgmsg_to_cv2(img, "rgb8")
        tracker = tracker_factory(
            data.track_type,
            img,
            [[data.start_pt.x, data.start_pt.y]],
            seg_color=data.seg_color,
        )
        self.non_error_trackers[data.cam_idx].append(
            {"id": data.id, "tracker": tracker}
        )

    def update_trackers(self, data):
        """Update trackers with a Lukas Kanade update given the new frame

        Args:
            data (Image): new ROS image message
        """
        if not self.ready:
            return

        camera_idx = int(data.header.frame_id)
        frame = self.bridge.imgmsg_to_cv2(img_msg=data, desired_encoding="rgb8")
        marked_up_image = np.copy(frame)

        formula: ErrorFunction
        for formula in self.error_formulas[camera_idx]:
            formula.update_tracking(frame)
            formula.draw(marked_up_image, thickness=10)
            msg: ErrorTrackedPoints = formula.get_msg()
            msg.cam_idx = camera_idx
            self.error_function_publishers[camera_idx].publish(msg)

        for tracker in self.non_error_trackers[camera_idx]:
            point = tracker["tracker"].update_tracking(frame)
            msg: IDPoint = IDPoint(id=tracker["id"], x=point[0][0], y=point[0][1])
            msg.header.stamp = data.header.stamp
            self.track_point_publishers[camera_idx].publish(msg)
            cv2.circle(
                marked_up_image, (int(point[0][0]), int(point[0][1])), 0, (255, 0, 0), 5
            )

        c_img_msg: CompressedImage = self.bridge.cv2_to_compressed_imgmsg(
            cvim=marked_up_image
        )

        img_msg: Image = self.bridge.cv2_to_imgmsg(
            cvim=marked_up_image, encoding="rgb8"
        )
        img_msg.header.stamp = rospy.get_rostime()
        self.tracking_publishers[camera_idx].publish(img_msg)
        self.compressed_tracking_publishers[camera_idx].publish(c_img_msg)

    def reset_all_tracking(self, data: Empty):
        """Resets all tracking, removing any currently running formulas for every camera."""
        for idx in self.error_formulas.keys():
            self.error_formulas[idx] = []

        for idx in self.non_error_trackers.keys():
            self.non_error_trackers[idx] = []

        self.cartesian_error_formulas = []

    def remove_tracking_by_id(self, data: String):
        """Removes any tracking related to the id given by the data

        Args:
            data (String): String ROS message
        """
        for value in self.error_formulas.values():
            formula: ErrorFunction
            for formula in value:
                if formula.get_id() == data.data:
                    value.remove(formula)

        for formula in self.cartesian_error_formulas:
            if formula.get_id() == data.data:
                self.cartesian_error_formulas.remove(formula)

        for value in self.non_error_trackers.values():
            for formula in value:
                if formula["id"] == data.data:
                    value.remove(formula)

    def swap_waypoint_by_id(self, req: SwapWaypointRequest):
        """Swaps the fixed point target (waypoint) for the given ID.

        This ONLY works on Waypoint tracking tasks, behavior is otherwise undefined.

        Args:
            data (String): String ROS msg containing id
        Returns:
            SwapWaypointResponse: service response.
        """
        for value in self.error_formulas.values():
            formula: ErrorFunction
            for formula in value:
                if formula.get_id() != req.id:
                    continue

                result = formula.swap_to_next_waypoint()
                return SwapWaypointResponse(
                    success=True, no_more_waypoints=(not result)
                )

        return SwapWaypointResponse(success=False)

    def publish_tracking_and_error(self, event=None):
        """Publish tracked points and error from ALL cameras combined."""
        # get error + tracked points for all cameras, aggregate, and publish
        error: Error = Error()
        error.error = []
        tracked_points: TrackedPoints = TrackedPoints()
        tracked_points.points = []

        # iterate for each camera
        for val in self.error_formulas.values():
            formula: ErrorFunction
            for formula in val:

                # get error from formula
                err = formula.get_error()
                for e in err:
                    error.error.append(e)

                # get all points for each formula
                points = formula.points
                for pt in points:
                    point = TrackedPoint()
                    point.x = pt[0]
                    point.y = pt[1]
                    tracked_points.points.append(point)

        for formula in self.cartesian_error_formulas:
            err = formula.get_error()
            for e in err:
                error.error.append(e)

        self.eef_pos_pub.publish(tracked_points)
        self.image_error_pub.publish(error)


def main(args):
    # this node seems to need a sleep to start properly in tmux, not sure why, TODO: try to fix.
    rospy.sleep(5)
    rospy.loginfo("Starting TrackingNode ...")
    node = TrackingNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down TrackingNode...")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
