from screeninfo import get_monitors
from tobii_stream_engine import GazePoint
from collections import namedtuple

ScreenCoordinate = namedtuple("ScreenCoordinate", ["x", "y"])


def coord_to_pixels(gaze_point: GazePoint):
    """
    Function to turn a gaze_point as specified in the Tobii Stream Engine API to a pixel
    screen coordinate.

    https://developer.tobii.com/product-integration/stream-engine/#tobii_streamsh

    If the coordinate is outside [0.0, 1.0], then the nearest pixel on the screen will be used.

    Note: If the gaze is outside the screen, then data will be narrowed in scope, and saccades
    off the screen will need to be captured in a different way. This function is for use for UI
    input, and may need to be expanded to include consideration for off-screen glances.

    @type gaze_point: GazePoint
    @param gaze_point: Gaze point as specified in Tobii Stream Engine API
    @rtype: ScreenCoordinate
    @return: The screen coordinate corresponding to gaze_point on the primary monitor
        - (0, 0) is left upper corner of the screen.
    """
    height: int = 0
    width: int = 0

    # Many monitors may be connected, assuming that the eye tracker is on the primary monitor
    for monitor in get_monitors():
        if (
            monitor.is_primary or monitor.is_primary is None
        ):  # If there are access control issues, run xhost + in the local (not Docker) terminal.
            height = monitor.height
            width = monitor.width

    x: int
    y: int

    if gaze_point.position_xy.x < 0.0:
        x = 0
    elif gaze_point.position_xy.x > 1.0:
        x = height
    else:
        x = round(gaze_point.position_xy.x * height)

    if gaze_point.position_xy.y < 0.0:
        y = 0
    elif gaze_point.position_xy.y > 1.0:
        y = width
    else:
        y = round(gaze_point.position_xy.y * width)

    screen_coordinate = ScreenCoordinate(x, y)

    return screen_coordinate
