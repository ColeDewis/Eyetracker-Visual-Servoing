# TODO
import cv2
import numpy as np
import rospy
import cv_bridge
import tf_conversions
import tf2_ros
from geometry_msgs.msg import Transform, TransformStamped, Quaternion
from sensor_msgs.msg import Image
from custom_msgs.msg import Contour

rospy.init_node("cnt_track")
bridge = cv_bridge.CvBridge()
contour_publisher = rospy.Publisher(
    "/tracking_node/cam6/contours", Contour, queue_size=10
)
contour_img_pub = rospy.Publisher("/cameras/cam6/contours", Image, queue_size=10)
tf_broadcaster = tf2_ros.TransformBroadcaster()


def frame_cb(frame: Image):
    img = bridge.imgmsg_to_cv2(frame, "rgb8")
    # im_bw = cv2.inRange(segmentation_image, id, id)  # id, id
    blurred = cv2.medianBlur(img, 17)
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)  # BGR

    # mask created with HSV thresholds
    filter_low_mask = (90, 35, 35)
    filter_high_mask = (133, 255, 255)
    mask = cv2.inRange(hsv, filter_low_mask, filter_high_mask)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return

    contour = max(contours, key=cv2.contourArea)

    contour = np.array(contour)
    min_el = min(contour, key=lambda x: x[0][1])
    min_idx = np.where(np.all(contour == min_el, axis=1))[0][0]
    contour = np.row_stack([contour[min_idx:], contour[:min_idx]])
    contour = np.unique(contour, axis=1)

    c = Contour(id=str(id), x_points=contour[:, 0, 0], y_points=contour[:, 0, 1])
    c.header.stamp = rospy.get_rostime()
    contour_publisher.publish(c)

    cnt_img = cv2.drawContours(img, [contour], 0, (255, 0, 0), 3)
    contour_img_pub.publish(bridge.cv2_to_imgmsg(cnt_img, "rgb8"))

    tf = TransformStamped()
    tf.header.frame_id = "end_effector_link"
    tf.child_frame_id = "cam7"
    tf.header.stamp = rospy.get_rostime()
    tf.transform.translation.x = 0.0
    tf.transform.translation.y = -0.056
    tf.transform.translation.z = 0.0
    q_mat = tf_conversions.transformations.euler_matrix(0, 0, 0)
    q_rot = tf_conversions.transformations.quaternion_from_matrix(q_mat)
    tf.transform.rotation = Quaternion(x=q_rot[0], y=q_rot[1], z=q_rot[2], w=q_rot[3])
    tf_broadcaster.sendTransform(tf)


frame_sub = rospy.Subscriber("/cameras/cam6", Image, frame_cb)
rospy.spin()
