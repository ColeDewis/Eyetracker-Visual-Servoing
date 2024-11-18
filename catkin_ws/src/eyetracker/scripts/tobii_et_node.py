#!/usr/bin/python3.10
import rospy

from tobii_stream_engine import (
    Api,
    Device,
    GazePoint,
    EyePosition,
    Stream,
    GazeOrigin,
    GazePoint,
    Stream,
    UserPresence,
    get_api_version,
)

import eyetracker.msg as eyemsg
from geometry_msgs.msg import Point
from eyetracker.parse_tobii_api import coord_to_pixels
from typing import TypedDict


class TobiiEyeTrackerData(TypedDict):
    gaze_point: GazePoint
    gaze_origin: GazeOrigin
    eye_position: EyePosition
    user_presence: UserPresence
    timestamp: int


eye_tracker_data: TobiiEyeTrackerData = {
    "gaze_point": None,
    "gaze_origin": None,
    "eye_position": None,
    "user_presence": None,
    "timestamp": 0,
}

prev_gaze_point = []


class TobiiEyeTracker:
    def __init__(self):
        """Initialize the eye tracker."""

        rospy.init_node("tobii_eye_tracker", disable_signals=True)

        ### Creating objects for the framework.
        tobiiAPI = Api()

        ### Fetching fails later if devices are not checked
        device_urls = tobiiAPI.enumerate_local_device_urls()

        if not len(device_urls):
            print("No device found")
            Exception(
                ValueError
            )  # TODO: Change error type here to be more descriptive - no device found

        ### Subscribe to the different data streams for the Tobii Eye Tracker 5
        device = Device(api=tobiiAPI, url=device_urls[0])
        supported_capabilities = device.get_supported_capabilities()
        supported_streams = device.get_supported_streams()

        if Stream.GAZE_POINT in supported_streams:
            device.subscribe_gaze_point(callback=self.on_gaze_point)

        if Stream.GAZE_ORIGIN in supported_streams:
            device.subscribe_gaze_origin(callback=self.on_gaze_origin)

        if Stream.EYE_POSITION_NORMALIZED in supported_streams:
            device.subscribe_eye_position(callback=self.on_eye_position)

        if Stream.USER_PRESENCE in supported_streams:
            device.subscribe_user_presence(callback=self.on_user_presence)

        self.gaze_point_publisher = rospy.Publisher(
            "/eyetracker/gaze_point", eyemsg.GazePoint, queue_size=10
        )
        self.raw_gaze_point_publisher = rospy.Publisher(
            "/eyetracker/gaze_point_raw", eyemsg.GazePoint, queue_size=10
        )
        self.gaze_origin_publisher = rospy.Publisher(
            "/eyetracker/gaze_origin", eyemsg.GazeOrigin, queue_size=10
        )
        self.eye_position_publisher = rospy.Publisher(
            "/eyetracker/eye_position", eyemsg.EyePosition, queue_size=10
        )
        self.user_presence_publisher = rospy.Publisher(
            "/eyetracker/user_presence", eyemsg.UserPresence, queue_size=10
        )

        ### Validated publishers

        self.valid_gaze_point_publisher = rospy.Publisher(
            "/eyetracker/gaze_point_valid", eyemsg.ValidGazePoint, queue_size=10
        )
        self.valid_raw_gaze_point_publisher = rospy.Publisher(
            "/eyetracker/gaze_point_raw_valid", eyemsg.ValidGazePoint, queue_size=10
        )
        self.valid_gaze_origin_publisher = rospy.Publisher(
            "/eyetracker/gaze_origin_valid", eyemsg.ValidGazeOrigin, queue_size=10
        )
        self.valid_eye_position_publisher = rospy.Publisher(
            "/eyetracker/eye_position_valid", eyemsg.ValidEyePosition, queue_size=10
        )
        self.valid_user_presence_publisher = rospy.Publisher(
            "/eyetracker/user_presence_valid", eyemsg.ValidUserPresence, queue_size=10
        )

        ### Start the device
        ### NOTE: Subscriptions must run BEFORE the device is started, else no data will be captured.
        rospy.loginfo(f"Starting Eye Tracker: {device.get_device_info()}")
        rospy.loginfo(f"Supported Streams: {device.get_supported_streams()}")

        try:
            device.run()
        except KeyboardInterrupt:
            rospy.loginfo(f"Shutting down...")
            del device

    def on_gaze_point(self, timestamp: int, gaze_point: GazePoint) -> None:
        """
        Callback for GazePoint subscription.

        @type timestamp: int
        @param timestamp: timestamp for when @param data has been updated.
            NOTE: this is a value from the beginning of the program runtime and as such
                can only be used to calculate deltas.
                Refer to the Tobii Stream Engine documentation for more information.
            https://developer.tobii.com/product-integration/stream-engine/#tobii_streamsh

        @type data: GazePoint
        @param data: GazePoint point which has been updated at delta timestamp.
                    Individual parameters of GazePoint can be found here:
                        https://developer.tobii.com/product-integration/stream-engine/#tobii_streamsh

        """
        pixels = coord_to_pixels(gaze_point)
        eye_tracker_data.update({"gaze_point": gaze_point, "timestamp": timestamp})

        message = eyemsg.GazePoint()
        now = rospy.get_rostime()
        message.header.stamp = now
        message.validity = gaze_point.validity
        message.x = pixels.x
        message.y = pixels.y
        self.gaze_point_publisher.publish(message)

        message_raw = eyemsg.GazePoint()
        message_raw.header.stamp = now
        message_raw.validity = gaze_point.validity
        message_raw.x = gaze_point.position_xy.x
        message_raw.y = gaze_point.position_xy.y
        self.raw_gaze_point_publisher.publish(message)

        if gaze_point.validity:
            message = eyemsg.ValidGazePoint()
            now = rospy.get_rostime()
            message.header.stamp = now
            message.x = pixels.x
            message.y = pixels.y
            self.valid_gaze_point_publisher.publish(message)

        return

    def on_gaze_origin(self, timestamp: int, gaze_origin: GazeOrigin) -> None:
        """
        Callback for GazeOrigin subscription. Updates current eye_tracker_data object.

        @type timestamp: int
        @param timestamp: timestamp for when @param data has been updated.
            NOTE: this is a value from the beginning of the program runtime and as such
                can only be used to calculate deltas.
                Refer to the Tobii Stream Engine documentation for more information.
            https://developer.tobii.com/product-integration/stream-engine/#tobii_streamsh

        @type data: GazeOrigin
        @param data: GazeOrigin object which has been updated at delta timestamp.
                    Individual parameters of GazeOrigin can be found here:
                        https://developer.tobii.com/product-integration/stream-engine/#tobii_streamsh

        """
        eye_tracker_data.update({"gaze_origin": gaze_origin, "timestamp": timestamp})

        message = eyemsg.GazeOrigin()
        message.header.stamp = rospy.get_rostime()
        message.left_validity = gaze_origin.left_validity
        message.right_validity = gaze_origin.right_validity
        message.left_xyz = Point(
            x=gaze_origin.left_xyz.x,
            y=gaze_origin.left_xyz.y,
            z=gaze_origin.left_xyz.z,
        )
        message.right_xyz = Point(
            x=gaze_origin.right_xyz.x,
            y=gaze_origin.right_xyz.y,
            z=gaze_origin.right_xyz.z,
        )
        self.gaze_origin_publisher.publish(message)

        if gaze_origin.left_validity and gaze_origin.right_validity:
            message = eyemsg.ValidGazeOrigin()
            message.header.stamp = rospy.get_rostime()
            message.left_xyz = Point(
                x=gaze_origin.left_xyz.x,
                y=gaze_origin.left_xyz.y,
                z=gaze_origin.left_xyz.z,
            )
            message.right_xyz = Point(
                x=gaze_origin.right_xyz.x,
                y=gaze_origin.right_xyz.y,
                z=gaze_origin.right_xyz.z,
            )

            self.valid_gaze_origin_publisher.publish(message)
        return

    def on_eye_position(self, timestamp: int, eye_position: EyePosition) -> None:
        """
        Callback for EyePosition subscription.

        @type timestamp: int
        @param timestamp: timestamp for when @param data has been updated.
            NOTE: this is a value from the beginning of the program runtime and as such
                can only be used to calculate deltas.
                Refer to the Tobii Stream Engine documentation for more information.
            https://developer.tobii.com/product-integration/stream-engine/#tobii_streamsh

        @type data: EyePosition
        @param data: EyePosition object which has been updated at delta timestamp.
                    Individual parameters of EyePosition can be found here:
                        https://developer.tobii.com/product-integration/stream-engine/#tobii_streamsh

        """
        eye_tracker_data.update({"eye_position": eye_position, "timestamp": timestamp})

        message = eyemsg.EyePosition()
        message.header.stamp = rospy.get_rostime()
        message.left_validity = eye_position.left_validity
        message.right_validity = eye_position.right_validity
        message.left_eye_position = Point(
            x=eye_position.left_xyz.x,
            y=eye_position.left_xyz.y,
            z=eye_position.left_xyz.z,
        )
        message.right_eye_position = Point(
            x=eye_position.right_xyz.x,
            y=eye_position.right_xyz.y,
            z=eye_position.right_xyz.z,
        )

        self.eye_position_publisher.publish(message)

        if eye_position.left_validity and eye_position.right_validity:
            message = eyemsg.ValidEyePosition()
            message.header.stamp = rospy.get_rostime()
            message.left_eye_position = Point(
                x=eye_position.left_xyz.x,
                y=eye_position.left_xyz.y,
                z=eye_position.left_xyz.z,
            )
            message.right_eye_position = Point(
                x=eye_position.right_xyz.x,
                y=eye_position.right_xyz.y,
                z=eye_position.right_xyz.z,
            )
            self.valid_eye_position_publisher.publish(message)

        return

    def on_user_presence(self, timestamp: int, user_presence: UserPresence) -> None:
        """
        Callback for UserPresence subscription.

        @type timestamp: int
        @param timestamp: timestamp for when @param data has been updated.
            NOTE: this is a value from the beginning of the program runtime and as such
                can only be used to calculate deltas.
                Refer to the Tobii Stream Engine documentation for more information.
            https://developer.tobii.com/product-integration/stream-engine/#tobii_streamsh

        @type data: UserPresence
        @param data: UserPresence object which has been updated at delta timestamp.
                    Individual parameters of UserPresence can be found here:
                        https://developer.tobii.com/product-integration/stream-engine/#tobii_streamsh

        """
        eye_tracker_data.update(
            {"user_presence": user_presence, "timestamp": timestamp}
        )
        message = eyemsg.UserPresence()
        message.header.stamp = rospy.get_rostime()
        if user_presence == UserPresence.UNKNOWN:
            message.status = 0
        elif user_presence == UserPresence.AWAY:
            message.status = 1
        else:  # UserPresence.PRESENT
            message.status = 2

        self.user_presence_publisher.publish(message)

        if user_presence == UserPresence.PRESENT:
            message = eyemsg.ValidUserPresence()
            message.header.stamp = rospy.get_rostime()
            message.status = 2

            self.valid_user_presence_publisher.publish(message)

        return


def main():
    TobiiEyeTracker()


if __name__ == "__main__":
    main()
