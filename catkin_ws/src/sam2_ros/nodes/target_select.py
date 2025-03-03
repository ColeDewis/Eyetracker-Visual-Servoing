#!/usr/bin/python3
import sys
import rospy
import cv_bridge
import cv2
import random
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from custom_msgs.srv import Sam2Prompt, Sam2PromptRequest
from custom_msgs.msg import Point2D, MaskArray
from std_msgs.msg import Empty

class Sam2TargetSelect:
    def __init__(self):
        self.last_frame = None
        self.cv_br = cv_bridge.CvBridge()
        self.imsub = rospy.Subscriber("/camera/color/image_raw", Image, self.img_cb)
        self.reqsrv = rospy.ServiceProxy("/sam2/prompt", Sam2Prompt)
        self.reset_pub = rospy.Publisher("/sam2/reset", Empty, queue_size=10)
        self.start_pub = rospy.Publisher("/uvs/start", Empty, queue_size=10)

    def img_cb(self, msg: Image):
        self.last_frame = self.cv_br.imgmsg_to_cv2(msg)

    def send_target(self):
        plt.imshow(self.last_frame)
        point = plt.ginput(3)

        req = Sam2PromptRequest()
        req.obj_id = 3 #random.randint(0, 20)
        req.labels = [1, 1, 0]
        req.points = [Point2D(x=point[0][0], y=point[0][1]), Point2D(x=point[1][0], y=point[1][1]), Point2D(x=point[2][0], y=point[2][1])]
        plt.close()
        self.reqsrv.wait_for_service()
        self.reqsrv.call(req)

        msg = rospy.wait_for_message("/sam2/masks", MaskArray)
        mask = msg.masks[0]
        mask = self.cv_br.imgmsg_to_cv2(mask, "8UC1")
        cv2.imshow("returned mask", mask)
        key = cv2.waitKey()
        if key == ord('y'):
            self.start_pub.publish(Empty())
        else:
            self.reset_pub.publish(Empty())

if __name__ == "__main__":
    rospy.init_node("sam2_target_selector")

    print(sys.argv)
    node = Sam2TargetSelect()
    while node.last_frame is None:
        rospy.sleep(0.1)

    node.send_target()