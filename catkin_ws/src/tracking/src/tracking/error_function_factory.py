from custom_msgs.msg import ErrorDefinition, TrackComponent, DistanceDefinition
from ws_utils.enums import ErrorDefinitionType, TrackComponentType, TrackerType
from tracking.error_functions import *
import tracking.distance_functions as dist_f


def error_function_factory(
    definition: ErrorDefinition,
    starting_frame: list,
    dist_valid: bool,
) -> ErrorFunction:
    """Factory method for an error functions. Takes in a definition and returns the appropriate ErrorFunction

    Args:
        definition (ErrorDefinition): _description_
        starting_frame (list): _description_
        dist_valid (bool): _description_

    Returns:
        ErrorFunction: error function created from the definition
    """
    err_type = definition.type

    if err_type == ErrorDefinitionType.POINT_POINT:
        return __create_point_to_point_constraint(
            definition, starting_frame, dist_valid
        )

    elif err_type == ErrorDefinitionType.POINT_LINE:
        return __create_point_to_line_constraint(definition, starting_frame, dist_valid)

    elif err_type == ErrorDefinitionType.LINE_LINE:
        return __create_line_to_line_constraint(definition, starting_frame, dist_valid)

    elif err_type == ErrorDefinitionType.POINT_CONIC:
        return __create_point_to_conic_constraint(
            definition, starting_frame, dist_valid
        )

    elif err_type == ErrorDefinitionType.WAYPOINTS:
        return __create_waypoint_constraint(definition, starting_frame, False)

    elif err_type == ErrorDefinitionType.IBVS_PT_PT:
        return __create_ibvs_point_to_point_constraint(definition, starting_frame)

    elif err_type == ErrorDefinitionType.IBVS_WAYPOINTS:
        return __create_waypoint_constraint(definition, starting_frame, True)


def __create_point_to_point_constraint(
    definition: ErrorDefinition, starting_frame: list, dist_valid: bool
) -> PointToPoint:
    """Create a point to point constraint object from a distance definition

    Args:
        definition (ErrorDefinition): error definition object to create the constraint from
        starting_frame (list): image array to initialize with
        dist_valid (bool): whether the distance definition is valid and should be used

    Returns:
        PointToPoint: point to point constraint object
    """
    components = definition.components
    distance_definition: DistanceDefinition = definition.distance_info

    # assume we have 2 points
    p1 = np.array(
        [components[0].points[0].x, components[0].points[0].y], dtype=np.float32
    )
    p2 = np.array(
        [components[1].points[0].x, components[1].points[0].y], dtype=np.float32
    )

    dist_ref_pts = None

    # we assume distance should be applied to the second point
    if dist_valid:
        dist_ref_pts = [np.copy(p2)]
        p2 = dist_f.update_targets_with_distance(distance_definition, [p2])[0]

    source_track_type = components[0].track_type
    target_track_type = components[1].track_type

    seg_color = (
        components[0].seg_color
        if source_track_type == TrackerType.SEGMENTATION
        else components[1].seg_color
    )

    return PointToPoint(
        definition.id,
        starting_frame,
        p1,
        p2,
        source_track_type,
        target_track_type,
        definition.task_scale,
        dist_ref_pts,
        seg_color=seg_color,
    )


def __create_waypoint_constraint(
    definition: ErrorDefinition, starting_frame: list, ibvs: bool
) -> Waypoints:
    """Create a waypoints constraint from an Error definition

    Args:
        definition (ErrorDefinition): error definition to create constraint from
        starting_frame (list): image array to initialize with

    Returns:
        Waypoints: waypoints constraint object
    """
    components = definition.components

    wypt_idx = 0 if components[0].type == TrackComponentType.WAYPOINTS else 1
    pt_idx = 0 if wypt_idx == 1 else 1

    waypoints = []
    for point in components[wypt_idx].points:
        waypoints.append([point.x, point.y])

    track_pt = np.array(
        [components[pt_idx].points[0].x, components[pt_idx].points[0].y],
        dtype=np.float32,
    )
    source_track_type = components[pt_idx].track_type
    waypoint_track_type = components[wypt_idx].track_type

    return Waypoints(
        definition.id,
        starting_frame,
        track_pt,
        np.asarray(waypoints, dtype=np.float32),
        source_track_type,
        waypoint_track_type,
        ibvs,
        definition.task_scale,
    )


def __create_ibvs_point_to_point_constraint(
    definition: ErrorDefinition, starting_frame: list
) -> IBVSPointToPoint:
    """Create an IBVS point to point constraint object from a distance definition

    Args:
        definition (ErrorDefinition): error definition object to create the constraint from
        starting_frame (list): image array to initialize with
        dist_valid (bool): whether the distance definition is valid and should be used

    Returns:
        IBVSPointToPoint: ibvs point to point constraint object
    """
    components = definition.components

    # assume we have 2 points
    # assume point 2 is the target for now
    track = np.array(
        [components[0].points[0].x, components[0].points[0].y], dtype=np.float32
    )
    target = np.array(
        [components[1].points[0].x, components[1].points[0].y], dtype=np.float32
    )
    source_track_type = components[0].track_type
    target_track_type = components[1].track_type

    return IBVSPointToPoint(
        definition.id,
        starting_frame,
        track,
        target,
        source_track_type,
        target_track_type,
        task_scale=definition.task_scale,
    )


def __create_point_to_line_constraint(
    definition: ErrorDefinition, starting_frame: list, dist_valid: bool
) -> PointToLine:
    """Create a point to line constraint object from a distance definition

    Args:
        definition (ErrorDefinition): error definition object to create the constraint from
        starting_frame (list): image array to initialize with
        dist_valid (bool): whether the distance definition is valid and should be used

    Returns:
        PointToLine: point to line constraint object
    """
    components = definition.components
    distance_definition = definition.distance_info
    dist_ref_pts = None

    pt_idx = 0 if components[0].type in TrackComponentType.ANY_POINT else 1
    ln_idx = 0 if pt_idx == 1 else 1

    # first one is point
    pt = [components[pt_idx].points[0].x, components[pt_idx].points[0].y]
    lp1 = [components[ln_idx].points[0].x, components[ln_idx].points[0].y]
    lp2 = [components[ln_idx].points[1].x, components[ln_idx].points[1].y]
    source_track_type = components[pt_idx].track_type
    target_track_type = components[ln_idx].track_type

    # we assume line is the target and thus distance applies to it.
    if dist_valid:
        dist_ref_pts = [np.copy(lp1), np.copy(lp2)]
        lp1, lp2 = dist_f.update_targets_with_distance(distance_definition, [lp1, lp2])

    return PointToLine(
        definition.id,
        starting_frame,
        np.array(pt, dtype=np.float32),
        np.array([lp1, lp2], dtype=np.float32),
        source_track_type,
        target_track_type,
        definition.task_scale,
        dist_ref_pts,
    )


def __create_line_to_line_constraint(
    definition: ErrorDefinition, starting_frame: list, dist_valid: bool
) -> LineToLine:
    """Create a line to line constraint object from a distance definition

    Args:
        definition (ErrorDefinition): error definition object to create the constraint from
        starting_frame (list): image array to initialize with
        dist_valid (bool): whether the distance definition is valid and should be used

    Returns:
        LineToLine: line to line constraint object
    """
    components = definition.components
    distance_definition = definition.distance_info
    dist_ref_pts = None

    # assume we have 2 lines
    p1 = [components[0].points[0].x, components[0].points[0].y]
    p2 = [components[0].points[1].x, components[0].points[1].y]
    p3 = [components[1].points[0].x, components[1].points[0].y]
    p4 = [components[1].points[1].x, components[1].points[1].y]

    source_track_type = components[0].track_type
    target_track_type = components[1].track_type

    # assume distance is applied to the second line
    if dist_valid:
        dist_ref_pts = [np.copy(p3), np.copy(p4)]
        p3, p4 = dist_f.update_targets_with_distance(distance_definition, [p3, p4])

    return LineToLine(
        definition.id,
        starting_frame,
        np.array([p1, p2], dtype=np.float32),
        np.array([p3, p4], dtype=np.float32),
        source_track_type,
        target_track_type,
        definition.task_scale,
        dist_ref_pts,
    )


def __create_point_to_conic_constraint(
    definition: ErrorDefinition, starting_frame: list, dist_valid: bool
) -> PointToEllipse:
    """Create a line to line constraint object from a distance definition

    Args:
        definition (ErrorDefinition): error definition object to create the constraint from
        starting_frame (list): image array to initialize with
        dist_valid (bool): whether the distance definition is valid and should be used

    Returns:
        PointToEllipse: point to ellipse constraint object
    """
    components = definition.components
    distance_definition = definition.distance_info

    pt_idx = 0 if components[0].type in TrackComponentType.ANY_POINT else 1
    cn_idx = 0 if pt_idx == 1 else 1
    pt = np.array(
        [components[pt_idx].points[0].x, components[pt_idx].points[0].y],
        dtype=np.float32,
    )
    conic_data = components[cn_idx].points
    source_track_type = components[pt_idx].track_type
    target_track_type = components[cn_idx].track_type

    conic_pts = []
    data: Point2D
    for data in conic_data:
        conic_pts.append([data.x, data.y])

    dist_ref_pts = None
    if dist_valid:
        dist_ref_pts = [np.copy(p1), np.copy(p2)]
        p1, p2 = dist_f.update_targets_with_distance(distance_definition, [p1, p2])

    return PointToEllipse(
        definition.id,
        starting_frame,
        pt,
        np.array(conic_pts, dtype=np.float32),
        source_track_type,
        target_track_type,
        definition.task_scale,
        dist_ref_pts,
    )
