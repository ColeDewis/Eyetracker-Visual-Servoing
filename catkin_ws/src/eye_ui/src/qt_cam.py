import rospy
from custom_msgs.msg import Point2D
from sensor_msgs.msg import Image
import cv_bridge
from PyQt5 import QtWidgets
from PyQt5.QtGui import (
    QImage,
    QPixmap,
)
from PyQt5.QtCore import (
    QThread,
    pyqtSignal as Signal,
    pyqtSlot as Slot,
)
import cv2
import imutils
import sys
import numpy as np


class SubscriberCamera(QThread):
    frame_signal = Signal(np.ndarray)

    def __init__(self):
        super().__init__()
        self.b = cv_bridge.CvBridge()
        self.im_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.frame_cb, queue_size=10
        )
        self.im_shape = None
        self.started = False

    def run(self):
        self.started = True
        rospy.spin()

    def cvimage_to_label(self, image):
        cv2.circle(image, (320, 240), 5, (0, 0, 255), -1)
        image = imutils.resize(image, height=960)  # TODO: figure out better resize
        return image

    def frame_cb(self, msg: Image):
        if self.started:
            frame = self.b.imgmsg_to_cv2(msg, "rgb8")
            self.im_shape = frame.shape
            frame = self.cvimage_to_label(frame)
            self.frame_signal.emit(frame)


class CvCameraThread(QThread):
    frame_signal = Signal(np.ndarray)

    def __init__(self):
        super().__init__()
        self.im_shape = None

    def run(self):
        self.cap = cv2.VideoCapture(0)
        while self.cap.isOpened():
            _, frame = self.cap.read()
            self.im_shape = frame.shape
            frame = self.cvimage_to_label(frame)
            self.frame_signal.emit(frame)

    def cvimage_to_label(self, image):
        image = imutils.resize(image, height=960)  # TODO: figure out better resize
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image


class MainApp(QtWidgets.QMainWindow):
    def __init__(self, cam_mode: str = "subscriber"):
        super().__init__()
        self.cam_mode = cam_mode
        self.unnormalized_target_sub = rospy.Subscriber(
            "/eyetracker/vs_target/unnormalized",
            Point2D,
            self.target_callback,
            queue_size=10,
        )
        self.normalized_target_pub = rospy.Publisher(
            "/eyetracker/vs_target", Point2D, queue_size=10
        )
        self.init_ui()
        self.show()

    def init_ui(self):
        self.setWindowTitle("Eyetrack GUI")

        widget = QtWidgets.QWidget(self)

        layout = QtWidgets.QVBoxLayout()
        widget.setLayout(layout)

        self.im_label = QtWidgets.QLabel()
        layout.addWidget(self.im_label)

        self.open_btn = QtWidgets.QPushButton("Start Camera", clicked=self.open_camera)
        layout.addWidget(self.open_btn)

        if self.cam_mode == "subscriber":
            self.camera_thread = SubscriberCamera()
        else:
            self.camera_thread = CvCameraThread()
        self.camera_thread.frame_signal.connect(self.setImage)

        self.setCentralWidget(widget)

        self.latest_target = [0, 0]
        self.first = True

    def open_camera(self):
        if self.camera_thread.isRunning():
            self.close()
        else:
            self.camera_thread.start()
            self.open_btn.setText("Quit")

    def target_callback(self, target: Point2D):
        self.latest_target = np.array([target.x, target.y])

    @Slot(np.ndarray)
    def setImage(self, image):
        self.im_label.resize(image.shape[1], image.shape[0])
        im = image.copy()
        im = QImage(im, im.shape[1], im.shape[0], QImage.Format_RGB888)
        self.im_label.setPixmap(QPixmap.fromImage(im))

        shape = self.camera_thread.im_shape
        scale_y = image.shape[0] / shape[0]
        scale_x = image.shape[1] / shape[1]

        # NOTE: these offsets might be wrong, but theres some wasted space difference b/w the overlay and the image window
        self.latest_target[1] -= 9 + 37
        self.latest_target[0] -= 9

        # NOTE clip is bad, if out of range just shouldn't publish
        self.latest_target[0] = self.latest_target[0] / scale_x
        self.latest_target[1] = self.latest_target[1] / scale_y

        x_in_range = self.latest_target[0] > 0 and self.latest_target[0] < shape[1]
        y_in_range = self.latest_target[1] > 0 and self.latest_target[1] < shape[0]

        if x_in_range and y_in_range:
            target = Point2D(x=self.latest_target[0], y=self.latest_target[1])
            self.normalized_target_pub.publish(target)


if __name__ == "__main__":
    rospy.init_node("eyetrack_gui")
    app = QtWidgets.QApplication([])
    main_window = MainApp(cam_mode="subscriber")
    main_window.showMaximized()
    sys.exit(app.exec())
