from PyQt5.QtCore import Qt
import sys
from PyQt5 import QtWidgets
from PyQt5.QtGui import (
    QPaintEvent,
    QPainter,
    QPen,
    QColor,
)
from PyQt5.QtCore import (
    Qt,
    QThread,
    QTimer,
    QPointF,
    pyqtSignal as Signal,
    pyqtSlot as Slot,
)
import numpy as np
import rospy
from eyetracker.msg import GazePoint, ClassifiedGazePoint
from custom_msgs.msg import Point2D


class EyetrackThread(QThread):
    gaze_signal = Signal(np.ndarray)
    class_gaze_signal = Signal(list)

    def __init__(self):
        super().__init__()

        self.eye_sub = rospy.Subscriber(
            "/eyetracker/gaze_point",
            GazePoint,
            self.gaze_cb,
            queue_size=10,
        )
        self.classified_eye_sub = rospy.Subscriber(
            "/eyetracker/classified_gaze_point",
            ClassifiedGazePoint,
            self.class_gaze_cb,
            queue_size=10,
        )

    def run(self):
        rospy.spin()

    def gaze_cb(self, data: GazePoint):
        if data.validity:
            self.gaze_signal.emit(np.array([data.x, data.y]))

    def class_gaze_cb(self, data: ClassifiedGazePoint):
        if data.validity:
            self.class_gaze_signal.emit([np.array([data.x, data.y]), data.type])


class MainApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.gaze_pub = rospy.Publisher(
            "/eyetracker/vs_target/unnormalized",
            Point2D,
            queue_size=10,
        )
        self.setWindowFlags(
            Qt.WindowTransparentForInput
            | Qt.WindowDoesNotAcceptFocus
            | Qt.FramelessWindowHint
            | Qt.WindowStaysOnTopHint
        )
        self.setAttribute(Qt.WA_TranslucentBackground, True)
        self.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        self.setGeometry(70, 27, 1920 - 70, 1080 - 27)
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Eyetracker Pane")

        # TODO: might not be any point of this being a thread atm
        self.eye_thread = EyetrackThread()
        self.eye_thread.gaze_signal.connect(self.drawEyePos)
        self.eye_thread.class_gaze_signal.connect(self.drawClassEyePos)
        self.eye_thread.start()

        self.last_eye = np.zeros(2)
        self.last_class = None
        self.drawEyePos(self.last_eye)

    @Slot(np.ndarray)
    def drawEyePos(self, pos):
        self.last_eye = pos

        # 70 x 27 is offset not occupied
        self.gaze_pub.publish(
            Point2D(x=(self.last_eye[0] - 70), y=(self.last_eye[1] - 27))
        )
        self.update()

    @Slot(list)
    def drawClassEyePos(self, pos):
        # ends up being bad if we actually use the points we get from this for visualization,
        # but the color visualization for classifying is nice to have available
        self.last_class = pos[1]

    def paintEvent(self, a0: QPaintEvent) -> None:
        painter = QPainter()
        painter.begin(self)
        painter.setPen(QPen(QColor(255, 255, 255, 120), 6))
        painter.drawEllipse(
            QPointF(self.last_eye[0] - 70, self.last_eye[1] - 27), 20, 20
        )

        if self.last_class == "saccade":
            painter.setPen(QPen(Qt.red, 3))
        elif self.last_class == "pursuit":
            painter.setPen(QPen(Qt.blue, 3))
        elif self.last_class == "fixation":
            painter.setPen(QPen(Qt.green, 3))
        else:
            painter.setPen(QPen(Qt.black, 3))

        painter.drawEllipse(
            QPointF(self.last_eye[0] - 70, self.last_eye[1] - 27), 20, 20
        )
        painter.drawText(
            QPointF(self.last_eye[0] - 35 - 70, self.last_eye[1] + 35 - 27),
            f"{self.last_eye[0]}, {self.last_eye[1]}",
        )
        painter.end()
        return super().paintEvent(a0)


if __name__ == "__main__":
    rospy.init_node("eyetracker_pane")
    app = QtWidgets.QApplication([])
    main_window = MainApp()
    main_window.showMaximized()

    # def func():
    #     main_window.last_eye = [np.random.randint(0, 1920), np.random.randint(0, 1080)]
    #     # main_window.update()
    #     print(main_window.geometry())

    # timer = QTimer()
    # timer.timeout.connect(func)
    # timer.start(1000)

    sys.exit(app.exec())
