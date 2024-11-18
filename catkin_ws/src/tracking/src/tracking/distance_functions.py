import numpy as np

from custom_msgs.msg import DistanceDefinition

def update_targets_with_distance(distance_info: DistanceDefinition, static_pts):
    """Calculates and moves the points given by static points to the positions defined by distance_info

    Args:
        distance_info (DistanceDefinition): object that gives distance definition information
        static_pts (list): fixed points we want to move

    Returns:
        list: new points
    """
    direction = distance_info.direction
    p1, p2, p3, p4 = distance_info.plane_points
    
    # We make the assumption that the points defined clockwise
    p1 = np.array([p1.x, p1.y, 1])
    p2 = np.array([p2.x, p2.y, 1])
    p3 = np.array([p3.x, p3.y, 1])
    p4 = np.array([p4.x, p4.y, 1])
    
    # calculate vanishing point 1
    l1 = np.cross(p1, p2)
    l2 = np.cross(p3, p4)
    v1 = np.cross(l1, l2) 
    v1 = v1 / v1[2]
    
    # calculate vanishing point 2
    l3 = np.cross(p1, p4)
    l4 = np.cross(p2, p3)
    v2 = np.cross(l3, l4)
    v2 = v2 / v2[2]
    
    # vanishing line
    vl = np.cross(v1, v2)
    
    new_pts = []
    # for each point, we have to extend it out
    for point in static_pts:
        a1 = np.array([point[0], point[1], 1])
        if distance_info.direction in (0, 2):
            # get reference distance in the direction we want
            D = distance_info.reference_distance_u
            
            # calculate p_t based on parallel assumption
            search_line = np.cross(a1, v1)
            v3_finder = np.cross(p4, a1)
            v3 = np.cross(v3_finder, vl)
            pt_finder = np.cross(p3, v3)
            p_t = np.cross(search_line, pt_finder)
            p_t = p_t / p_t[2]
            
            # set the proper vanishing point
            v = v1
            
        elif distance_info.direction in (1, 3):
            # get deference distance in the direction we want 
            D = distance_info.reference_distance_v
            
            # calculate p_t based on parallel assumption
            search_line = np.cross(a1, v2)
            v3_finder = np.cross(p3, a1)
            v3 = np.cross(v3_finder, vl)
            pt_finder = np.cross(p2, v3)
            p_t = np.cross(search_line, pt_finder)
            p_t = p_t / p_t[2]
            
            # set the proper vanishing point
            v = v2
            
        # NOW: segment a-p_t is the same as our reference distance p3-p4.
        # we have formulated our problem now as a search along line search_line, where our points that we use are:
        # 1) a, 2) p_t, 3) a+epsilon, 4) p*, 5) v1/v2
        # a - reference point for constraint
        # p_t =known
        # a + epsiolon known
        # p* we solve for
        # v1/v2 is vanishing point of parallel defined lines
        epsilon = 0.01 # epsilon is a distance along our line we define a point away from a1, so we have a 5th point for our math. 
        # TODO: we may not need epsilon, do some testing?
        
        # determine cr1
        d_a1_pt = np.linalg.norm(a1[:2] - p_t[:2])
        d_aeps_v = np.linalg.norm(a1[:2] - v[:2]) + epsilon # increase distance by epsilon to avoid zeros
        d_pt_aeps = np.linalg.norm(a1[:2] - p_t[:2]) + epsilon
        d_a1_v = np.linalg.norm(a1[:2] - v[:2])
        cr1 = (d_a1_pt * d_aeps_v) / (d_pt_aeps * d_a1_v)
        
        # determine distances related to cr2
        d_a1_s2 = np.linalg.norm(a1[:2] - p_t[:2])
        d_pt_v = np.linalg.norm(p_t[:2] - v[:2])
        
        d_star = distance_info.desired_distance
        
        D_star = (d_star * d_a1_s2) / ((D / cr1) * d_pt_v)
        
        # p if moving towards vanishing point
        p_towards = (d_a1_v * D_star) / (D_star + 1)
        
        # p if moving away from vanishing point
        p_away = (d_a1_v * D_star) / (D_star - 1)
        
        # get the unit vector between the point "a" and point "v"
        unit_vec = [(v[0] - a1[0])/d_aeps_v, (v[1] - a1[1])/d_aeps_v]
        
        # depending on the direction, we now need to move along the vector.
        # however, the direction depends on a) what direction we want and b) which direction the line towards
        # the vanishing point actually leads us
        if direction == 0: # positive x
            if v[0] > a1[0]: # vanishing point goes right, follow it
                p_img = [a1[0] + unit_vec[0] * p_towards, a1[1] + unit_vec[1] * p_towards]
            else: # vanishing point goes left, we want positive, so we go negative.
                p_img = [a1[0] + unit_vec[0] * p_away, a1[1] + unit_vec[1] * p_away]
        elif direction == 1: # positive y
            if v[1] > a1[1]: # vanishing point down, don't follow
                p_img = [a1[0] + unit_vec[0] * p_away, a1[1] - unit_vec[1] * p_away]
            else: # vanishing point up, follow
                p_img = [a1[0] + unit_vec[0] * p_towards, a1[1] + unit_vec[1] * p_towards]
        elif direction == 2: # negative x
            if v[0] > a1[0]: # vanishing point left, want positive, so go right
                p_img = [a1[0] + unit_vec[0] * p_away, a1[1] + unit_vec[1] * p_away]
            else: # vanishing point goes left, we follow it
                p_img = [a1[0] + unit_vec[0] * p_towards, a1[1] + unit_vec[1] * p_towards]
        elif direction == 3: # negative y
            if v[1] > a1[1]: # vanishing point down, follow
                p_img = [a1[0] + unit_vec[0] * p_towards, a1[1] + unit_vec[1] * p_towards]
            else: # vanishing point up, don't follow
                p_img = [a1[0] + unit_vec[0] * p_away, a1[1] + unit_vec[1] * p_away]
        
        new_pts.append(p_img)
        
    return np.asarray(new_pts)