from collections import deque
import numpy as np


class IVDTClassifier:
    """Gaze Classifier using I-VDT, intended for real-time use."""

    def __init__(self, win_size: int, vel_thres: float, disp_thresh: float):
        self.temp_window = deque()
        self.win_size = win_size
        self.vel_thres = vel_thres
        self.disp_thresh = disp_thresh
        self.last_point = None
        self.in_disp_win = False

    def add_point(self, point: np.ndarray):
        # NOTE: this seems kind of problematic, since we cannot classify in realtime: only in the size of the temporal window
        # which will induce a delay of whatever the window size is in being able to classify a fixation or smooth pursuit
        # probably delay of ~100ms

        # NOTE: the rostopic for this runs at about 20hz, while the standard stream runs at about 30hz. Honestly not bad

        if self.last_point is None:
            self.last_point = point
            return None, None

        velocity = np.linalg.norm(point - self.last_point)
        self.last_point = point

        if velocity > self.vel_thres:
            # point is a saccade

            # NOTE: when we saccade away, we throw away the temporal window if it was incomplete.
            self.temp_window.clear()

            return point, "saccade"

        self.temp_window.append(point)

        # if we don't have enough points to determine dispersion, nothing to say for now
        if len(self.temp_window) < self.win_size:
            return None, None

        disp, centroid = self.__calculate_window_dispersion(self.temp_window)

        if disp < self.disp_thresh:
            # we can continue adding points to the window to see if the fixation continues
            self.in_disp_win = True
            return centroid, "fixation"
        elif self.in_disp_win:
            # we were in a window, but now are not meeting the threshold.
            # The newest point is then a smooth pursuit, and we can throw away the window
            sp = self.temp_window.pop()
            self.temp_window.clear()
            self.in_disp_win = False
            return sp, "pursuit"
        else:
            # we do not have enough for a fixation, so the first point in the window can be marked as a smooth pursuit
            sp = self.temp_window.popleft()
            return sp, "pursuit"

    def __calculate_window_dispersion(self, window: deque):
        # dispersion = [max(x) - min(x)] + [max(y) - min(y)]
        window = np.array(window)
        max_x, max_y = np.max(window, axis=0)
        min_x, min_y = np.min(window, axis=0)
        centroid = np.sum(window, axis=0) / window.shape[0]

        return (max_x - min_x) + (max_y - min_y), centroid
