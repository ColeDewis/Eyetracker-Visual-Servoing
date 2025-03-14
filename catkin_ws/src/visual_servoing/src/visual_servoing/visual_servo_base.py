from abc import ABC

class VisualServoBase(ABC):
    def __init__(self):
        """Initialize the Visual Servoing object
        """
        pass

    def initialize(self):
        """Initialize jacobian
        """
        pass

    def update(self):
        """Update jacobian
        """
        pass



# what will these look like
# - UVS: to init, needs the shape
# - IBVS: needs the shape, but also the control transform
# not really sure i can make a base class but would be nice to organize this better
