#!/usr/bin/python3.10
import rospy
import cv_bridge
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from custom_msgs.msg import MaskArray
from custom_msgs.srv import Sam2Prompt, Sam2PromptRequest, Sam2PromptResponse
import rospkg
import torch
import numpy as np
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

# using point prompt
# points = np.array([[670, 247]], dtype=np.float32)
# # for labels, `1` means positive click and `0` means negative click
# labels = np.array([1], dtype=np.int32)
# bbox = np.array([[600, 214], [765, 286]], dtype=np.float32)

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
        self.tracking_init = False
        self.last_frame = None
        
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=10)
        self.mask_pub = rospy.Publisher("/sam2/masks", MaskArray, queue_size=10)
        self.mask_debug = rospy.Publisher("/sam2/debug", Image, queue_size=10)

        self.request_srv = rospy.Service("/sam2/prompt", Sam2Prompt, self.prompt_callback)
        
        self.br = cv_bridge.CvBridge()
        rospy.loginfo("Started Sam2 Node!")
        
    def init_tracking(self) -> bool:
        """Initialize tracking using the last frame we got from cameras

        Returns:
            bool: true if successful, false otherwise
        """
        if self.last_frame is None:
            rospy.logerr("No camera frame has been received yet, cannot initialize.")
            return False

        self.predictor.load_first_frame(self.last_frame)
        self.tracking_init = True
        return True
        
    def prompt_callback(self, req: Sam2PromptRequest) -> Sam2PromptResponse:
        """Callback for a sam2 prompt request

        Args:
            req (Sam2PromptRequest): prompt reqest message
        """

        # NOTE: for now assume we apply any prompt on the latest frame we have. 
        # this will likely incur some delay but should be good enough
        resp = Sam2PromptResponse()
        resp.success = True

        # check for tracking init, and try to initialize it if not.
        if not self.tracking_init and not self.init_tracking():
            resp.success = False
            resp.err = "Tracking could not be initialized"
            return resp

        focus_points = np.array([[point.x, point.y] for point in req.points])
        labels = np.array([label for label in req.labels], dtype=np.int32)
        rospy.loginfo(f"{self.predictor.frame_idx}, len: {self.predictor.condition_state['images']}")

        if not self.model_init:
            self.predictor.add_new_prompt(
                frame_idx=0, 
                obj_id=req.obj_id, 
                points=focus_points, 
                labels=labels,
                clear_old_points=False
            ) 
        else:
            # NOTE: this seems to work but I had to hack some stuff together from the library
            # is_new_id = req.obj_id not in self.predictor.condition_state["obj_ids"]
            
            # TODO: later i would like to make this functional, but for now it breaks really bad
            # SAM2 doesn't like adding new object ids during tracking, and this method below seemed to be
            # an attempt of a workaround (they hard set tracking started back to false) in the library, but it doesn't work for me.
            # If is_new_id is set to true. 
            # One way I can think to workaround this is to just grab all existing masks from last frame, reset tracking,
            # and re-initialize using those masks, plus the new point.
            # Since the video frames are not all stored, we can't just use the other method either. This is amittedly an easy
            # fix in the source code, but isn't good for memory. I think resetting and reinitializing may actually be the 
            # best option, given that we shouldn't incur this cost too frequently.
            is_new_id = False
            self.predictor.add_new_prompt_during_track(
                obj_id=req.obj_id, point=focus_points, labels=labels, if_new_target=is_new_id
            ) 
 
        self.model_init = True
        return resp

        # def add_new_prompt(
        # self,
        # frame_idx,
        # obj_id,
        # points=None,
        # labels=None,
        # bbox=None,
        # clear_old_points=True,
        # normalize_coords=True,
        # ):
    

    def image_callback(self, image: Image) -> None:
        """Callback for new image message, updates SAM2 tracking if initialized

        Args:
            image (Image): image message to update with
        """
        im = self.br.imgmsg_to_cv2(image, "rgb8")
        self.last_frame = im
        
        if not self.model_init:
            return
        
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
            out_obj_ids, out_mask_logits = self.predictor.track(im)
            self.publish_masks(out_obj_ids, out_mask_logits)

    def publish_masks(self, ids, logit_masks) -> None:
        """Publishes output masks from SAM2

        Args:
            ids (list): list of ids from sam2
            logit_masks (list): list of corresponding masks
        """
        # rospy.loginfo(f"IDS: {ids}")
        # rospy.loginfo(f"Logit masks: {logit_masks}")
        masks = MaskArray()
        masks.header.stamp = rospy.get_rostime()
        masks.masks = []
        for i, mask in enumerate(logit_masks):
            imgmsg: Image = self.br.cv2_to_imgmsg((mask > 0.0).float().cpu().numpy().reshape((480, 640)).astype(np.uint8) * 255, encoding="8UC1") 
            imgmsg.header.frame_id = f"{ids[i]}"
            masks.masks.append(imgmsg)

            if i == 0:
                self.mask_debug.publish(imgmsg)

        self.mask_pub.publish(masks)

    def reset(self, msg: Empty) -> None:
        """Resets tracking on an empty message

        Args:
            msg (Empty): empty (trigger) msg
        """
        self.predictor.reset_state()
        self.model_init = False


if __name__ == "__main__":

    rospy.init_node("sam2_node")
    node = Sam2Node()
    # while node.last_frame is None:
    #     rospy.sleep(0.1)
    # node.init_tracking(None)

    rospy.spin()