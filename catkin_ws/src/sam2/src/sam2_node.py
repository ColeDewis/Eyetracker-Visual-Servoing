#!/usr/bin/python3.10

import torch
import rospy
from sam2.build_sam import build_sam2_camera_predictor
from sam2.build_sam import build_sam2

checkpoint = "./checkpoints/sam2_hiera_large.pt"
model_cfg = "sam2_hiera_l.yaml"
predictor = build_sam2_camera_predictor(model_cfg, checkpoint)

# cap = cv2.VideoCapture(<your video or camera >)
cap = None

if_init = False

# OPTIONS: either A: use the sam2-realtime repo (doesnt support 2.1) or B: write my own script.
# the video option is more ideal since we have proper tracking, but not having 2.1 might be a bit sad

with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        width, height = frame.shape[:2][::-1]

        if not if_init:
            predictor.load_first_frame(frame)
            if_init = True
            _, out_obj_ids, out_mask_logits = predictor.add_new_prompt(None)

        else:
            out_obj_ids, out_mask_logits = predictor.track(frame)
            ...
