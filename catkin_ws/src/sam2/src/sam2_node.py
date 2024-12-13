import rospy
import cv_bridge
from sensor_msgs.msg import Image
import rospkg
import torch
import cv2
from sam2.build_sam import build_sam2_camera_predictor

sam2_checkpoint = "../checkpoints/sam2.1_hiera_small.pt"
model_cfg = "configs/sam2.1/sam2.1_hiera_s.yaml"
predictor = build_sam2_camera_predictor(model_cfg, sam2_checkpoint)

cap = cv2.VideoCapture(0)

if_init = False

with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        width, height = frame.shape[:2][::-1]

        if not if_init:
            predictor.load_first_frame(frame)
            if_init = True
            _, out_obj_ids, out_mask_logits = predictor.add_new_prompt(<your promot >)

        else:
            out_obj_ids, out_mask_logits = predictor.track(frame)
            ...
            
class Sam2Node:
    def __init__(self):
        
        sam2_checkpoint = rospy.get_param("~checkpoint", default="checkpoints/sam2.1_hiera_small.pt")
        model_cfg = rospy.get_param("~conifg", default="config/sam2.1_hiera_s.yaml")
        
        pkg = rospkg.RosPack()
        path = pkg.get_path("sam2_node")
        sam2_checkpoint = f"{path}/{sam2_checkpoint}"
        model_cfg = f"{path}/{model_cfg}"
        
        self.predictor = build_sam2_camera_predictor(model_cfg, sam2_checkpoint)
        self.model_init = False
        
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=10)
        self.mask_pub = rospy.Publisher("/sam2/mask", Image, queue_size=10)
        
        self.br = cv_bridge.CvBridge()
        
    def init_tracking(self, msg):
        # TODO: initialize tracking with sam2
        pass
    
    def image_callback(self, image: Image):
        im = self.br.imgmsg_to_cv2(image, "rgb8")
        
        if self.model_init:
            out_obj_ids, out_mask_logits = self.predictor.track(im)


if __name__ == "__main__":
    rospy.init_node("sam2_node")
    rospy.spin()