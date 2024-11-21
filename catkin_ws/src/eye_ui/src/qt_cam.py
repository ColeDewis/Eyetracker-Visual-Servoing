import rospy
from eyetracker.msg import GazePoint
from custom_msgs.msg import Point2D
from sensor_msgs.msg import Image
import cv_bridge
from PyQt5 import QtWidgets
from PyQt5.QtGui import (
    QImage,
    QMouseEvent,
    QPaintEvent,
    QPixmap,
    QPainter,
    QPen,
)
from PyQt5.QtCore import (
    Qt,
    QThread,
    QPointF,
    pyqtSignal as Signal,
    pyqtSlot as Slot,
)
import cv2
import imutils
import sys
import numpy as np


class EyetrackThread(QThread):
    gaze_signal = Signal(np.ndarray)

    def __init__(self):
        super().__init__()

        self.eye_sub = rospy.Subscriber(
            "/eyetracker/gaze_point", GazePoint, self.gaze_cb, queue_size=10
        )

    def run(self):
        rospy.spin()

    def gaze_cb(self, data: GazePoint):
        if data.validity:
            self.gaze_signal.emit(np.array([data.x, data.y]))


class CvCameraThread(QThread):
    frame_signal = Signal(np.ndarray)

    def __init__(self):
        super().__init__()
        self.width = 0
        self.height = 0
        self.b = cv_bridge.CvBridge()
        self.im_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.frame_cb, queue_size=10
        )

    def set_resize_params(self, h, w):
        self.height = h
        self.width = w
        print(self.height)

    def run(self):
        rospy.spin()

    def cvimage_to_label(self, image):
        image = imutils.resize(image, height=960)  # TODO: figure out better resize
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image

    def frame_cb(self, msg: Image):
        frame = self.b.imgmsg_to_cv2(msg, "rgb8")
        frame = self.cvimage_to_label(frame)
        self.frame_signal.emit(frame)


class SubscriberCamera(QThread):
    frame_signal = Signal(np.ndarray)

    def __init__(self):
        super().__init__()
        self.width = 0
        self.height = 0

    def set_resize_params(self, h, w):
        self.height = h
        self.width = w
        print(self.height)

    def run(self):
        self.cap = cv2.VideoCapture(0)
        while self.cap.isOpened():
            _, frame = self.cap.read()
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
        self.init_ui()
        self.show()

        self.vs_target_pub = rospy.Publisher(
            "/eyetracker/vs_target", Point2D, queue_size=10
        )

    def init_ui(self):
        self.setWindowTitle("Eyetrack GUI")

        widget = QtWidgets.QWidget(self)

        layout = QtWidgets.QVBoxLayout()
        widget.setLayout(layout)

        self.im_label = QtWidgets.QLabel()
        self.eye_label = QtWidgets.QLabel()
        layout.addWidget(self.im_label)

        self.open_btn = QtWidgets.QPushButton("Start Camera", clicked=self.open_camera)
        layout.addWidget(self.open_btn)

        if self.cam_mode == "subscriber":
            self.camera_thread = SubscriberCamera()
        else:
            self.camera_thread = CvCameraThread()
        self.camera_thread.frame_signal.connect(self.setImage)

        # TODO: might not be any point of this being a thread atm
        self.eye_thread = EyetrackThread()
        self.eye_thread.gaze_signal.connect(self.drawEyePos)
        self.eye_thread.start()

        self.setCentralWidget(widget)

        self.last_eye = [0, 0]
        self.eye_label.setGeometry(self.im_label.geometry())
        layout.addWidget(self.eye_label)
        print(self.eye_label.geometry(), self.im_label.geometry())

    def open_camera(self):
        self.camera_thread.set_resize_params(self.height(), self.width())
        self.camera_thread.start()
        print(self.camera_thread.isRunning())

    @Slot(np.ndarray)
    def setImage(self, image):
        self.im_label.resize(image.shape[1], image.shape[0])
        self.eye_label.setGeometry(self.im_label.geometry())
        self.eye_label.raise_()

        im = image.copy()
        im = QImage(im, im.shape[1], im.shape[0], QImage.Format_RGB888)
        self.im_label.setPixmap(QPixmap.fromImage(im))

    @Slot(np.ndarray)
    def drawEyePos(self, pos):
        self.last_eye = pos
        # NOTE: for now we manually scale back to 640 x 480 but later need to use subscribed image sizes
        self.vs_target_pub.publish(
            Point2D(x=(self.last_eye[0] - 79) / 2, y=(self.last_eye[1] - 73) / 2)
        )

        # print(self.geometry())  # 1850 x 1016
        # print(pos)

    def paintEvent(self, a0: QPaintEvent) -> None:
        pixmap = QPixmap(self.eye_label.size())
        pixmap.fill(Qt.transparent)
        painter = QPainter(pixmap)
        painter.setPen(QPen(Qt.white, 3))
        # screen is 1850 x 1016 so have to subtract 70 and 64, then also 9 more for each
        painter.drawEllipse(
            QPointF(self.last_eye[0] - 79, self.last_eye[1] - 73), 20, 20
        )
        painter.drawText(
            QPointF(self.last_eye[0] - 79 - 30, self.last_eye[1] - 73 + 35),
            f"{self.last_eye[0] - 79}, {self.last_eye[1] - 73}",
        )
        painter.end()
        self.eye_label.setPixmap(pixmap)
        return super().paintEvent(a0)


if __name__ == "__main__":
    rospy.init_node("eyetrack_gui")
    app = QtWidgets.QApplication([])
    main_window = MainApp(cam_mode="subscriber")
    main_window.showMaximized()
    sys.exit(app.exec())
