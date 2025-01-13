#!/usr/bin/python3.10
import rospy
import cv_bridge
from sensor_msgs.msg import Image
import rospkg
import torch
from sam2.sam2_camera_predictor import SAM2CameraPredictor
from sam2.build_sam import build_sam2_camera_predictor

# sam2_checkpoint = "../checkpoints/sam2.1_hiera_small.pt"
# model_cfg = "configs/sam2.1/sam2.1_hiera_s.yaml"
# predictor = build_sam2_camera_predictor(model_cfg, sam2_checkpoint)


# with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
#         width, height = frame.shape[:2][::-1]

#         if not if_init:
#             predictor.load_first_frame(frame)
#             if_init = True
#             _, out_obj_ids, out_mask_logits = predictor.add_new_prompt(<your promot >)

#         else:
#             out_obj_ids, out_mask_logits = predictor.track(frame)

class Sam2Node:
    def __init__(self):
        
        sam2_checkpoint = rospy.get_param("~checkpoint", default="checkpoints/sam2.1_hiera_small.pt")
        model_cfg = rospy.get_param("~conifg", default="sam2.1_hiera_s.yaml")
        
        pkg = rospkg.RosPack()
        path = pkg.get_path("sam2_ros")
        sam2_checkpoint = f"{path}/{sam2_checkpoint}"
        # model_cfg = f"{path}/{model_cfg}"
        model_cfg = f"configs/sam2.1/{model_cfg}"
        
        self.predictor: SAM2CameraPredictor = build_sam2_camera_predictor(model_cfg, sam2_checkpoint)
        self.model_init = False
        self.last_frame = None
        
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=10)
        self.mask_pub = rospy.Publisher("/sam2/mask", Image, queue_size=10)
        
        self.br = cv_bridge.CvBridge()
        rospy.loginfo("Started Sam2 Node!")
        
    def init_tracking(self, msg):
        # TODO: initialize tracking with sam2
        if self.last_frame is None:
            rospy.logerr("No camera frame has been received yet, cannot initialize.")
        
        self.predictor.load_first_frame(self.last_frame)
        _, out_obj_ids, out_mask_logits = self.predictor.add_new_prompt() # TODO
        self.model_init = True
        pass
    
    def image_callback(self, image: Image):
        im = self.br.imgmsg_to_cv2(image, "rgb8")
        self.last_frame = im
        
        if not self.model_init:
            return
        
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
            out_obj_ids, out_mask_logits = self.predictor.track(im)

            # TODO convert to imgmsg, for each obj id
            for i, mask in enumerate(out_mask_logits):
                ...

if __name__ == "__main__":
    rospy.init_node("sam2_node")
    node = Sam2Node()
    rospy.spin()