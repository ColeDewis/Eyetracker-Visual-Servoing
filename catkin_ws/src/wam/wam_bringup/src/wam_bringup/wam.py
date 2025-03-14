import time
from math import degrees, radians
import tf_conversions
import numpy as np
import roboticstoolbox as rbt
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from std_msgs.msg import Bool
from wam_model import WamModel
from wam_msgs.msg import RTJointVel
from wam_srvs.srv import JointMove, JointMoveRequest, JointMoveResponse, PoseMove, PoseMoveRequest, PoseMoveResponse

# #####
# Limits for WAM
# In degree TODO find these!

JOINT_LIMIT = {
    0: [radians(-180.0), radians(180.0)],
    1: [radians(-128.9), radians(128.9)],
    2: [radians(-180.0), radians(180.0)],
    3: [radians(-147.8), radians(147.8)],
    4: [radians(-180.0), radians(180.0)],
    5: [radians(-120.3), radians(120.3)],
    6: [radians(-180.0), radians(180.0)],
}

JOINT_NAME_TO_ID = {
    "joint_1": 0,
    "joint_2": 1,
    "joint_3": 2,
    "joint_4": 3,
    "joint_5": 4,
    "joint_6": 5,
    "joint_7": 6,
}

class WAM(object):
    """WAM interface."""

    def __init__(
        self,
        robot_name: str = "wam",
        read_joint_state: bool = False,
    ):
        # ####################
        # Connect to WAM and setup publishers and subscribers
        # ####################
        # //Publishing the following rostopics
        # wam_joint_state_pub = n_.advertise < sensor_msgs::JointState > ("joint_states", 1); // wam/joint_states
        # wam_move_state_pub = n_.advertise < std_msgs::Bool > ("move_is_done", 1); // moving state
        # wam_pose_pub = n_.advertise < geometry_msgs::PoseStamped > ("pose", 1); // wam/pose

        # WAM Parameters
        # -----
        self.dof = 7    # TODO should allow for 7, 4, 2 ... 
        self.joint_names = [
            "FirstLink",
            "SecondLink",
            "ThirdLink",
            "FourthLink",
            "FifthLink",
            "SixthLink",
            "SeventhLink",
        ]
        self.joints = None
        self.pose = None
        self.is_moving = False
        self.joint_state_sub = rospy.Subscriber(
            "/wam/joint_states",
            JointState,
            self._joint_state_cb,
        )

        self.pose_sub = rospy.Subscriber(
            "/wam/pose",
            PoseStamped,
            self._pose_cb
        )
        self.moving_sub = rospy.Subscriber(
            "/wam/move_is_done",
            Bool,
            self._move_done_cb
        )

        self.wam_model = WamModel()

        # cart_vel_sub = n_.subscribe("cart_vel_cmd", 1, &WamNode::cartVelCB, this); // wam/cart_vel_cmd
        # ortn_vel_sub = n_.subscribe("ortn_vel_cmd", 1, &WamNode::ortnVelCB, this); // wam/ortn_vel_cmd
        # jnt_vel_sub = n_.subscribe("jnt_vel_cmd", 1, &WamNode::jntVelCB, this); // wam/jnt_vel_cmd
        # jnt_pos_sub = n_.subscribe("jnt_pos_cmd", 1, &WamNode::jntPosCB, this); // wam/jnt_pos_cmd
        # cart_pos_sub = n_.subscribe("cart_pos_cmd", 1, &WamNode::cartPosCB, this); // wam/cart_pos_cmd

        # //Advertising the following rosservices
        # gravity_srv = n_.advertiseService("gravity_comp", &WamNode::gravity, this); // wam/gravity_comp
        # go_home_srv = n_.advertiseService("go_home", &WamNode::goHome, this); // wam/go_home
        # hold_jpos_srv = n_.advertiseService("hold_joint_pos", &WamNode::holdJPos, this); // wam/hold_joint_pos
        # hold_cpos_srv = n_.advertiseService("hold_cart_pos", &WamNode::holdCPos, this); // wam/hold_cart_pos
        # hold_ortn_srv = n_.advertiseService("hold_ortn", &WamNode::holdOrtn, this); // wam/hold_ortn
        # joint_move_srv = n_.advertiseService("joint_move", &WamNode::jointMove, this); // wam/joint_move
        # pose_move_srv = n_.advertiseService("pose_move", &WamNode::poseMove, this); // wam/pose_move
        # cart_move_srv = n_.advertiseService("cart_move", &WamNode::cartMove, this); // wam/cart_pos_move
        # ortn_move_srv = n_.advertiseService("ortn_move", &WamNode::ortnMove, this); // wam/ortn_move

        # NOTE: any real time topics have a timeout of 0.3s 
        self.joint_vel_pub = rospy.Publisher(
            "/wam/jnt_vel_cmd",
            RTJointVel,
            queue_size=10
        )
        self.joint_pos_srv = rospy.ServiceProxy("/wam/joint_move", JointMove)
        self.pose_move_srv = rospy.ServiceProxy("/wam/pose_move", PoseMove)
        self.home_srv = rospy.ServiceProxy("/wam/go_home", Empty)

        rospy.wait_for_service("/wam/joint_move_srv")
        rospy.wait_for_service("/wam/go_home")

    def _joint_state_cb(self, msg: JointState):
        """Store joint angles inside the class instance."""
        self.joints = np.array(msg.position[: len(self.joint_names)]).astype(
            np.float32
        )

    def _pose_cb(self, msg: PoseStamped):
        self.pose = msg.pose

    def _move_done_cb(self, msg: Bool):
        self.is_moving = not msg.data

    def go_home(self):
        """Send WAM to default home position."""
        req = EmptyRequest()
        resp = self.home_srv.call(req)
        raise True

    def send_joint_angles(
        self,
        angles: list,
    ):
        """Move WAM to specified joint angles (radians).
        Args:
            angles: list, 7 DOF, in radians.
        """
        # Make sure angles is a numpy array
        if isinstance(angles, list):
            angles = np.array(angles)

        # Clip the degrees by joint_limit
        for i in range(len(angles)):
            angles[i] = np.clip(
                angles[i], a_min=JOINT_LIMIT[i][0], a_max=JOINT_LIMIT[i][1]
            )
        
        srv = JointMoveRequest()
        srv.joints = angles

        resp = self.joint_pos_srv.call(srv)

        return resp

    def send_joint_velocities(
        self,
        vels: list,
    ):
        """Set velocity for each individual joint.
        TODO: Might need to change max_rate to suit control frequency.
        """
        # Make sure angles is a numpy array
        if isinstance(vels, list):
            vels = np.array(vels)

        msg = RTJointVel()
        msg.velocities = vels
        self.joint_vel_pub.publish()

    def send_twist(
        self,
        vels: list,
    ):
        """Sends twist to the robot

        Args:
            vels (list): list of velocities (3 linear, 3 angular). Linear in m/s, angular deg/s
            duration_ms (int): twist command duration (millis)
        """
        raise NotImplementedError("No service for twist on WAM")

    def send_twist_topic(
        self,
        vels: list,
    ):
        """Sends twist to the robot, but using the /in topic instead of a service

        Args:
            vels (list): list of velocities (3 linear, 3 angular). Linear in m/s, angular deg/s
            duration_ms (int): twist command duration (millis)
            reference_frame (CartesianReferenceFrame, optional): reference frame for the twist. Defaults to CARTESIAN_REFERENCE_FRAME_BASE.
        """

        if isinstance(vels, list):
            vels = np.array(vels)
        
        # TODO: not sure if I need jacob0 or jacob0_analytical.
        # https://petercorke.github.io/robotics-toolbox-python/arm_superclass.html#roboticstoolbox.robot.Robot.Robot.jacob0
        j_0 = self.wam_model.jacob0(
            self.joints, end="bhand_base"
        )
        q_dot = np.linalg.pinv(j_0) @ vels

        self.send_joint_velocities(q_dot)

    def send_pose(
        self, x, y, z, theta_x, theta_y, theta_z
    ):
        """
        Sends a 3d pose to the wam

        Args:
            x (float): x target in m
            y (float): y target in m
            z (float): z target in m
            theta_x (float): x angle target in degrees
            theta_y (float): y angle target in degrees
            theta_z (float): z angle target in degrees

        Returns:
            bool: true/false depending on whether the command succeeded
        """
        srv = PoseMoveRequest()
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z

        q = tf_conversions.transformations.quaternion_from_euler(theta_x, theta_y, theta_z, "sxyz") # TODO might be wrong conversion
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        srv.pose = msg

        resp = self.pose_move_srv.call(srv)
        return resp

    def send_gripper_command(
        self,
        value: float,
    ):
        raise NotImplementedError("Gripper command not supported for WAM")
