from collections import deque
import numpy as np


class GazeClassifier:
    """Gaze Classifier using I-VDT, intended for real-time use."""

    def __init__(self, win_size: int, vel_thres: float, disp_thresh: float):
        self.buffer = deque(maxlen=win_size)
        self.last_dispersion_xv = np.zeros(2)  # min, max
        self.last_dispersion_yv = np.zeros(2)  # min, max

    def add_point(self, point: np.ndarray):
        if len(self.buffer) == self.buffer.maxlen:
            self.buffer.popleft()

        self.buffer.append(point)

    def update(self):
        # dispersion = [max(x) - min(x)] + [max(y) - min(y)]

        # pseudocode
        # 1. calc velocity threshold and determine if the new point was a saccade
        # 2. if not a saccade start trying to create a dispersion window
        #    keep adding points as long as dispersion remains low enough
        #    once it doesn't mark it as a smooth pursuit
        # 3. IF THE TEMPORAL WINDOW DOES NOT HAVE ENOUGH POINTS TO BE A DISPERSION
        #    NEED TO MARK AS SMOOTH PURSUIT AND THEN POP A POINT OUT
        #    (essentially need win_size minimim points to consider it a fixation)
        pass
