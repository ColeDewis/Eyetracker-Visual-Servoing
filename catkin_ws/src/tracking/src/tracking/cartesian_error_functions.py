import numpy as np
from abc import ABC, abstractmethod
from kortex_bringup import KinovaUtil

class CartesianErrorFunction(ABC):
    """Abstract CartesianErrorFunction class. Contains logic for calculating error."""
    
    def __init__(self, id, task_scale = 1.0):
        """Base constructor for error. Sets up tracking requirements and gets a color for annotation.

        Args:
            id (string): id for function identification
            task_scale (float): scaling for the task
        """
        self.id = id
        self.task_scale = task_scale
        
    def get_id(self) -> str:
        """Get the ID for the function.
        
        Returns:
            str: string id
        """
        return self.id
    
    @abstractmethod
    def get_error(self) -> list:
        """Get the error value. Must be implemented by subclass

        Returns:
            list: list containing the error. Since we could have both scalar or vector errors, we return
                them all as vector so that they can all be treated the same way in code by iteration. 
        """
        pass

class KinovaPoseConstriant(CartesianErrorFunction):
    """Error value based on error in one element of cartesian pose. Implements CartesianErrorFunction
    """
    def __init__(self, id: str, pose_element: int, desired_value: float, task_scale: float = 1.0):
        """Initialize a PoseConstraint, which measures error between a component of robot pose and a desired value.

        Args:
            id (str): identification string
            pose_element (int): 0 - 5 indexes for x, y, z, r, p, y elements of pose
            desired_value (float): desired value
            task_scale (float, optional): scaling for error value. Defaults to 1.0.
        """
        super().__init__(id, task_scale)
        self.pose_element = pose_element
        self.desired_value = desired_value
        
        self.gen3_util = KinovaUtil()
        
    def get_error(self) -> list:
        """Gets the error for the task, the difference between desired value and current value

        Returns:
            list: error value
        """
        pose = self.gen3_util.get_eef_pose()        
        return np.multiply([pose[self.pose_element] - self.desired_value], self.task_scale) if pose is not None else []
    
class KinovaJointAngleConstriant(CartesianErrorFunction):
    def __init__(self, id: str, joint_num: int, desired_value: float, task_scale: float = 1.0):
        """Initialize a PoseConstraint, which measures error between a component of robot pose and a desired value.

        Args:
            id (str): identification string
            joint_num (int): 0 - 6 indexes for each respective joint
            desired_value (float): desired value
            task_scale (float, optional): scaling for error value. Defaults to 1.0.
        """
        super().__init__(id, task_scale)
        self.pose_element = joint_num
        self.desired_value = desired_value
        self.gen3_util = KinovaUtil()
        
        
    def get_error(self) -> list:
        """Get the error, the difference in desired joint angle and the current one.

        Returns:
            list: error value
        """
        current_joints = self.gen3_util.get_arm_joints()
        return np.multiply([current_joints[self.joint_num] - self.desired_value], self.task_scale)