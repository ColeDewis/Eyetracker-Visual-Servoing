from numpy.typing import ArrayLike
from ws_utils.enums import TrackerType
from tracking.trackers import *


def tracker_factory(
    tracker_type: TrackerType,
    init_frame: ArrayLike,
    init_points: ArrayLike,
    seg_color=None,
) -> Tracker:
    """Factory Method for creating trackers.

    Args:
        tracker_type (TrackerType): type of tracker
        init_frame (ArrayLike): initial image
        init_points (ArrayLike): initial point position
        seg_mask (ArrayLike, optional): _description_. Defaults to None.

    Returns:
        Tracker: an initialized tracker object that can be updated with new frames.
    """

    if len(np.shape(init_points)) == 1:
        init_points = np.array([init_points])

    if tracker_type == TrackerType.LKPYR:
        return LKPyrTracker(init_frame, init_points)
    elif tracker_type == TrackerType.FIXED:
        return FixedTracker(init_frame, init_points)
    elif tracker_type == TrackerType.CAMSHIFT:
        return CamShiftTracker(init_frame, init_points)
    elif tracker_type == TrackerType.SEGCAMSHIFTLAYERED:
        return SegmentationCamShiftLayered(init_frame, init_points, debug=True)
    elif tracker_type == TrackerType.SEGMENTATION:
        return SegmentationTracker(init_frame, init_points, seg_color)
