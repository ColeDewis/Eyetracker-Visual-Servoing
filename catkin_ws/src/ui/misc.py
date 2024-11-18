import customtkinter as ctk
import tkinter as tk
from ws_utils.enums import TrackComponentType, ErrorDefinitionType
from utils.utils import *

# from scripts_widget import *
# from camera_utils.joystick_control import *
import rospy

# from joystick_widget import *
from joystick_control.ee_controller_dh import EE_controller_DH
from joystick_control.iris_control import IrisControl
from custom_msgs.msg import ExcavationFeedback, ErrorDefinition, ExcavationStart
from custom_msgs.srv import Clipu2netrInference, Clipu2netrInferenceRequest
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

from customtkinter import CTkFont as font
import cv2
from cv_bridge import CvBridge
import numpy as np
from PIL import Image as PILImage
import typing


class ScriptJoyTab(ctk.CTkFrame):
    def __init__(self, parent, root, kinova_present=False):
        ctk.CTkFrame.__init__(self, root)
        f = font(family="Helvetica", size=16, weight="normal")
        # self.script_handler = ScriptHandler(self)

        f = font(family="Helvetica", size=16, weight="normal")
        self.reuse_jacobians = tk.BooleanVar()

        # self.excav = ScriptTrigger(parent, self, "Excavation", ScriptNames.EXCAVATION, self.script_handler)
        self.excav = PublisherFromArgs(
            self,
            "/excavation/start",
            ExcavationStart,
            "Excavation",
            {"num_iterations": 12, "run_continuously": False, "reuse_jacobians": False},
            {"font": f, "width": 90, "height": 60},
        )
        # self.excav_cont = ScriptTrigger(parent, self, "Excavation\nContinuous", ScriptNames.EXCAVATION_CONTINUOUS, self.script_handler, button_color="#30bf5d", hover_color="#269649")
        self.excav_cont = PublisherFromArgs(
            self,
            "/excavation/start",
            ExcavationStart,
            "Excavation\nContinous",
            {
                "num_iterations": 12,
                "run_continuously": True,
                "reuse_jacobians": self.reuse_jacobians,
            },
            {"font": f, "width": 90, "height": 60},
        )
        # TODO excav_cont needs a truth value form the checkbox
        self.pause = TogglePause(parent, self)
        self.stop = EStop(parent, self)

        self.excav.grid(row=1, column=0, sticky="new", padx=10, pady=10)
        self.excav_cont.grid(row=1, column=1, sticky="new", padx=10, pady=10)
        ctk.CTkCheckBox(
            self, text="Reuse\nJacobians", variable=self.reuse_jacobians
        ).grid(column=1, row=2, sticky="n", padx=5, pady=5)
        self.pause.grid(row=1, column=2, sticky="new", padx=10, pady=10)
        self.stop.grid(row=1, column=3, sticky="new", padx=10, pady=10)

        self.parent = parent
        if kinova_present:
            self.joystick = JoystickInterface()

        joy_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.joy = ctk.CTkButton(
            joy_frame,
            width=90,
            height=60,
            font=f,
            text="Joystick\nStart",
            command=self.joystickOverride,
        )
        self.joy.grid(row=1, column=1, sticky="new", padx=5, pady=5)
        joy_frame.grid_columnconfigure((0, 2), weight=1)
        joy_frame.grid(row=1, column=4, sticky="new", padx=10, pady=10)
        self.joy_text = "Joystick Start"

        """self.chen_code = ChenCodeTrigger(self, parent).grid(
            column=5, row=1, sticky="new", padx=10, pady=10
        )"""
        self.delay = FeedbackDelayTrigger(self, self).grid(
            column=6, row=1, sticky="new", padx=10, pady=10
        )

        self.error_stuff = ScriptError(parent, self)
        self.error_stuff.grid(
            row=1, column=7, columnspan=5, sticky="new", padx=10, pady=10
        )

        self.grid_columnconfigure((0, 1, 2, 3, 4, 5, 6), weight=1)
        self.grid_columnconfigure((7), weight=5)

    def joystickOverride(self, *args):
        if self.joy_text == "Joystick Start":
            self.joystick_updater_sub = rospy.Subscriber(
                "/cameras/cam" + str(self.parent.camIndices[0]),
                Image,
                self.joystickUpdate,
            )
            self.joystick.allow_control(True)
            self.joy_text = "Joystick Cancel"
            self.parent.cancelServoing()

        else:
            self.parent.joystickOverride(False)
            self.joystick.allow_control(False)
            self.joy_text = "Joystick Start"
            self.joystick_updater_sub.unregister()
            self.joystick_updater_sub = None
        self.joy.configure(text=self.joy_text)

    def joystickDelay(self, delay):
        self.joystick.setDelay(delay)

    def joystickUpdate(self, *args):
        self.joystick.robot.step()


class ScriptError(ctk.CTkFrame):
    def __init__(self, parent, root):
        ctk.CTkFrame.__init__(self, root, fg_color="transparent")

        f = font(family="Helvetica", size=16, weight="normal")

        self.state = ctk.CTkLabel(self, text="State: Not Started", font=f)
        self.err = ctk.CTkLabel(self, text="Error: None", font=f)
        self.it = ctk.CTkLabel(self, text="Iteration: None", font=f)

        self.state.grid(column=0, row=0)
        self.err.grid(column=0, row=1)
        self.it.grid(column=0, row=2)

        self.exc_err = rospy.Subscriber(
            "/excavation/feedback", ExcavationFeedback, callback=self.config_mssg
        )
        self.last_state = ""

    def config_mssg(self, message):
        if message.cur_state == "waiting_for_servo":
            self.state.configure(
                text="State: " + message.cur_state + " - " + self.last_state
            )
        else:
            self.state.configure(text="State: " + message.cur_state)
            self.last_state = message.cur_state
        try:
            self.it.configure(text="Iteration: " + str(message.iteration_num))
            if message.vs_running:
                self.err.configure(text="Error: " + str(message.vs_error))
            else:
                self.err.configure(text="Error: N/A")
        except:
            pass


class JoystickInterface:
    def __init__(self):
        # rospy.init_node('iris_control', anonymous=True)
        joy = EE_controller_DH()
        # self.robot = IrisControl(controller=joy, init_camera=False)
        rate = rospy.Rate(10)  # 40hz

    def allow_control(self, state):
        self.robot.allow_control = state

    def setDelay(self, delay):
        self.robot.delay = delay


"""class ChenCodeTrigger(ctk.CTkFrame):
    def __init__(self, root, parent):
        ctk.CTkFrame.__init__(self, root, fg_color="transparent")
        self.prompt_entry = ctk.CTkTextbox(self, height=20)
        self.confirm_cam1 = ctk.CTkButton(
            self, width=90, text="Prompt - Cam1", command=self.sendPrompt1
        )
        self.confirm_cam2 = ctk.CTkButton(
            self, width=90, text="Prompt - Cam2", command=self.sendPrompt2
        )

        self.prompt_entry.grid(column=0, row=0, columnspan=2, padx=5, pady=5)
        self.confirm_cam1.grid(column=0, row=1, padx=5, pady=5)
        self.confirm_cam2.grid(column=1, row=1, padx=5, pady=5)
        self.parent = parent
        self.bridge = CvBridge()
        # self.text = ctk.CTkLabel(self)
        # self.text.grid(row=2, column=0)

    def sendPrompt(self, cam_num):
        base_img = self.parent.grabImage(cam_num)
        print(type(base_img))
        base_img_msg = Image()
        base_img_msg.header.stamp = rospy.Time.now()
        base_img_msg.height = base_img.height
        base_img_msg.width = base_img.width
        base_img_msg.encoding = "rgb8"
        base_img_msg.is_bigendian = False
        base_img_msg.step = 3 * base_img.width
        base_img_msg.data = np.array(base_img).tobytes()
        self.handleIm(base_img_msg)

    def handleIm(self, base_img):

        print(base_img.width, base_img.height)
        prompt = self.prompt_entry.get("0.0", "end")
        prompts = prompt.split(";")
        for i in range(0, len(prompts)):
            prompts[i] = prompts[i].strip()
        rospy.wait_for_service("/clipu2netr/inference")
        self.swap_waypoint_service = rospy.ServiceProxy(
            "/clipu2netr/inference", Clipu2netrInference
        )
        try:
            resp = self.swap_waypoint_service.call(
                Clipu2netrInferenceRequest(
                    phrases=prompts, image=base_img, confidence_threshold=0.5
                )
            )
            img = self.bridge.imgmsg_to_cv2(
                resp.segmented_image, desired_encoding="8UC1"
            )
            img_size = (len(img[0]), len(img))

            img = PILImage.fromarray(img)
            imgtk = ctk.CTkImage(img, size=img_size)
            print(type(imgtk))
            new_window = ctk.CTkToplevel()
            # self.text.configure(image=imgtk)
            ctk.CTkLabel(new_window, text="", image=imgtk).pack()

        except rospy.ServiceException as e:
            rospy.logerr(f"Swap Waypoint Service Failure: {e}")

    def sendPrompt1(self, *args):
        self.sendPrompt(str(self.parent.camIndices[0]))

    def sendPrompt2(self, *args):
        self.sendPrompt(str(self.parent.camIndices[-1]))
"""


class FeedbackDelayTrigger(ctk.CTkFrame):
    def __init__(self, root, parent):
        ctk.CTkFrame.__init__(self, root, fg_color="transparent")
        self.in_seconds = ctk.CTkComboBox(
            self, values=["0.0", "0.25", "0.5", "1.0", "2.0", "3.0", "4.0"]
        )
        self.confirm = ctk.CTkButton(self, text="Delay", command=self.sendDelay)
        self.in_seconds.grid(column=0, row=0, padx=5, pady=5)
        self.confirm.grid(column=0, row=1, padx=5, pady=5)
        self.parent = parent

    def sendDelay(self, *args):
        delay = float(self.in_seconds.get())
        self.parent.joystickDelay(delay)
