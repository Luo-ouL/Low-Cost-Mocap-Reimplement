import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt5.QtCore import QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLAxisItem

import serial
import time
import threading

from ui.user_interface_ui import Ui_MainWindow

from mocap.helpers import *

## Acquiring constants
def camera_initialization():
    # Acquire camera indicies
    while True:
        try:
            camera_indices = input("Please enter the cameras' indices(0,1...)\n")
            camera_indices = list(map(int, camera_indices.split()))
            # insure the length of the order is right
            if camera_indices[0] != 0:
                raise ValueError("Invalid input.")
            break
        except ValueError as e:
            print(f"Invalid input: {e}\nPlease enter a valid permutation.")
    # Cover camera-params[1] if necessary
    with open("mocap/camera-params_copy.json", "r") as f:
        camera_params_copy = json.load(f)
    with open("mocap/camera-params.json", "r") as f:
        camera_params = json.load(f)
    camera_params = camera_params_copy
    if len(camera_indices) == 2:
        camera_params[1] = camera_params_copy[camera_indices[1]]
        print("camera-params[1] is now covered.")
    with open("mocap/camera-params.json", "w") as f:
        json.dump(camera_params, f, indent=4)
    # Acquire camera_poses and camera_to_world_matrix
    if len(camera_indices) == 2:
        camera_poses = np.load(f"extrinsic/camera_poses{camera_indices[0]}{camera_indices[1]}.npy", allow_pickle=True)
    else:
        camera_poses = np.load(f"extrinsic/camera_poses.npy", allow_pickle=True)

    camera_to_world_matrix = np.load("extrinsic/camera_to_world_matrix.npy")

    return camera_poses, camera_to_world_matrix

camera_poses, camera_to_world_matrix = camera_initialization()
cameras = Cameras.instance()

np.set_printoptions(precision=8, suppress=True)

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

class VideoWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        # cameras initialization
        self.cameras = Cameras.instance()
        # Qtimer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.locate_object_points)
        self.timer.timeout.connect(self.update_setpoint_from_input)
        self.timer.timeout.connect(self.update_pid_from_input)
        self.timer.timeout.connect(self.update_trim_from_input)
        self.timer.timeout.connect(self.update_groundEff_from_input)
        self.timer.timeout.connect(self.update_trajectory_from_input)
        self.timer.start(10) # ms
        # view3D module
        axis = GLAxisItem()
        axis.setSize(x=1, y=1, z=1)
        self.view3D.addItem(axis)
        self.view3D.setCameraPosition(distance=0.5, azimuth=45, elevation=30)
        # blank point cloud
        self.scatter = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), color=(1, 1, 1, 1), size=5)
        self.view3D.addItem(self.scatter)
        # yaw visualization module
        self.yaw_line = gl.GLLinePlotItem(pos=np.zeros((2, 3)), color=(0, 1, 0, 1), width=2, mode='lines')
        self.view3D.addItem(self.yaw_line)
        # create serial_thread
        self.serial_thread = SerialThread(SERIAL_PORT, BAUD_RATE)
        self.serial_thread.data_received.connect(self.handle_serial_response)
        self.serial_thread.start()
        # enlarge module
        self.label_video.mousePressEvent = self.show_fullscreen_frame
        self.last_qt_image = None
        # localize drones module
        self.last_object_points = None
        self.last_object_time = 0
        self.current_object_points = None
        self.current_object_time = 0
        self.feature_dist = 0.16
        self.drone_pos = np.array([0, 0, 0, 0])
        # self.feature_dist = 0.095
        self.checkBox_locate_drones.setChecked(False)
        # setpoint module
        self.setpoint = np.array([0, 0, 0, 0])
        self.pushButton_PosSetpoint.clicked.connect(self.send_setpoint)
        # trajectory module
        self.trajectory_points = np.array([[0, 0, 0, 0]])
        self.expected_vel = 0
        self.pushButton_trajectory_on.clicked.connect(self.on_trajectory_module)
        self.pushButton_trajectory_off.clicked.connect(self.off_trajectory_module)
        self.trajectory_on = False
        # pid module
        self.pid = np.array([1, 0.2, 0.02, # xyPosPID
                             1.5, 0.1, 0.02, # zPosPID
                             0.3, 0.1, 0.05, # yawPosPID
                             0.5, 0.2, 0.1, # xyVelPID
                             0.3, 0.06, 0.06  # zVelPID
                             ])
        self.pushButton_pid.clicked.connect(self.send_pid)
        # trim module
        self.trim = np.array([0, 0, 0, 0])
        self.pushButton_trim.clicked.connect(self.send_trim)
        # groundEffect module
        self.groundEff = np.array([1200, 0])
        self.pushButton_groundEff.clicked.connect(self.send_groundEff)
        # arm status module
        self.armed = False
        self.pushButton_arm.clicked.connect(self.send_arm_status)
        # calibrate yaw module
        self.yaw_trim = 0
        self.pushButton_calibrate_yaw.clicked.connect(self.calibrate_yaw)
        # accuracy test module
        self.pushButton_test_accuracy.clicked.connect(self.test_accuracy)
        # quitbutton->close window
        self.pushButton_quit.clicked.connect(self.close)

    def locate_object_points(self):
        # localize object_points and update frames
        errors, object_points, frames = localize_object_points(camera_poses, camera_to_world_matrix)
        self.current_object_points = object_points
        if frames:
            frame = np.concatenate(frames, axis=1)
            rgb_image = cv.cvtColor(frame, cv.COLOR_BGR2RGB)  # convert to Qt recognizable
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.last_qt_image = qt_image
            self.label_video.setPixmap(QPixmap.fromImage(qt_image).scaled(
                self.label_video.size(), aspectRatioMode=1))  # scale propotionally
        # update 3D cloud
        if object_points is not None and len(object_points) > 0:
            self.scatter.setData(pos=np.array(object_points), color=(1, 0, 0, 1), size=5)
        # localize drones if necessary
        if self.checkBox_locate_drones.isChecked(): self.locate_drones()

    def locate_drones(self):
        if self.last_object_points is not None and self.current_object_points is not None:
            if len(self.current_object_points) == 3 and len(self.last_object_points) == 3:
                self.current_object_time = time.time()
                dt = self.current_object_time - self.last_object_time
                position, velocity = compute_pos_vel_yaw(
                    self.last_object_points, self.current_object_points, dt, self.yaw_trim, self.feature_dist)
                data_json = {
                    "pos": position.round(2).tolist(),
                    "vel": velocity.round(2).tolist()
                }
                data_byte = json.dumps(data_json).encode('UTF-8')
                self.serial_thread.send(data_byte)
                # update yaw
                if len(position) == 4:
                    centroid = position[:-1]
                    yaw = position[-1]
                    front = centroid + np.array([0.05*np.cos(yaw), 0.05*np.sin(yaw), 0])
                    line = np.vstack((centroid, front))
                    self.yaw_line.setData(pos=np.array(line))
                # visualize result
                self.label_xPos.setText(str(position[0].round(2)))
                self.label_yPos.setText(str(position[1].round(2)))
                self.label_zPos.setText(str(position[2].round(2)))
                self.label_yawPos.setText(str(position[3].round(2)))
                self.label_xVel.setText(str(velocity[0].round(2)))
                self.label_yVel.setText(str(velocity[1].round(2)))
                self.label_zVel.setText(str(velocity[2].round(2)))

                # for trajectory module
                self.drone_pos = position
        # update last_object_points
        self.last_object_points = self.current_object_points
        self.last_object_time = time.time()


    def update_setpoint_from_input(self):
        try:
            xPosSetpoint = self.lineEdit_xPosSetpoint.text()
            yPosSetpoint = self.lineEdit_yPosSetpoint.text()
            zPosSetpoint = self.lineEdit_zPosSetpoint.text()
            yawPosSetpoint = self.lineEdit_yawPosSetpoint.text()
            if xPosSetpoint and yPosSetpoint and zPosSetpoint and yawPosSetpoint:
                self.setpoint = np.array([float(xPosSetpoint), float(yPosSetpoint), float(zPosSetpoint), float(yawPosSetpoint)])
        except ValueError:
            print("Invalid setpoint format")

    def update_pid_from_input(self):
        try:
            xyPosP = self.lineEdit_xyPosP.text()
            xyPosI = self.lineEdit_xyPosI.text()
            xyPosD = self.lineEdit_xyPosD.text()
            zPosP = self.lineEdit_zPosP.text()
            zPosI = self.lineEdit_zPosI.text()
            zPosD = self.lineEdit_zPosD.text()
            yawPosP = self.lineEdit_yawPosP.text()
            yawPosI = self.lineEdit_yawPosI.text()
            yawPosD = self.lineEdit_yawPosD.text()
            xyVelP = self.lineEdit_xyVelP.text()
            xyVelI = self.lineEdit_xyVelI.text()
            xyVelD = self.lineEdit_xyVelD.text()
            zVelP = self.lineEdit_zVelP.text()
            zVelI = self.lineEdit_zVelI.text()
            zVelD = self.lineEdit_zVelD.text()
            if xyPosP and xyPosI and xyPosD and zPosP and zPosI and zPosD and yawPosP and yawPosI and yawPosD and xyVelP and xyVelI and xyVelD and zVelP and zVelI and zVelD:
                self.pid = np.array([float(xyPosP), float(xyPosI), float(xyPosD),
                                     float(zPosP), float(zPosI), float(zPosD),
                                     float(yawPosP), float(yawPosI), float(yawPosD),
                                     float(xyVelP), float(xyVelI), float(xyVelD),
                                     float(zVelP), float(zVelI), float(zVelD)])
        except ValueError:
            print("Invalid pid format")

    def update_trim_from_input(self):
        try:
            PitchTrim = self.lineEdit_PitchTrim.text()
            RollTrim = self.lineEdit_RollTrim.text()
            ThrottleTrim = self.lineEdit_ThrottleTrim.text()
            YawTrim = self.lineEdit_YawTrim.text()
            if PitchTrim and RollTrim and ThrottleTrim and YawTrim:
                self.trim = np.array([int(PitchTrim), int(RollTrim), int(ThrottleTrim), int(YawTrim)])
        except ValueError:
            print("Invalid trim fromat")
        return

    def update_groundEff_from_input(self):
        try:
            groundCoef = self.lineEdit_groundCoef.text()
            groundOffset = self.lineEdit_groundOffset.text()
            if groundCoef and groundOffset:
                self.groundEff = np.array([float(groundCoef), float(groundOffset)])
        except ValueError:
            print("Invalid groundEff fromat")
        return

    def update_trajectory_from_input(self):
        text = self.textEdit_waypoints.toPlainText()
        lines = text.strip().split("\n")
        waypoints = []
        for line in lines:
            if line.strip() == '':
                continue
            try:
                values = list(map(float, line.strip().split()))
            except ValueError:
                # print("Invalid waypoints input (not all numbers):", line)
                return
            if len(values) != 4:
                # print("Invalid waypoints input (must have 4 values):", line)
                return
            waypoints.append(values)
        if not waypoints:
            return
        waypoints = np.array(waypoints)
        # print('Parsed waypoints:')
        # print(waypoints)

        expected_vel_text = self.lineEdit_expectedVel.text().strip()
        if not expected_vel_text:
            return
        try:
            self.expected_vel = float(expected_vel_text)
        except ValueError:
            # print("Invalid expected velocity input.")
            return

        if self.expected_vel != 0:
            self.trajectory_points = generate_trajectory_points(waypoints, self.expected_vel)
            # print('Updated trajectory points:')
            # print(self.trajectory_points)
        else:
            print("Expected velocity must be non-zero.")


    def send_setpoint(self):
        setpoint = self.setpoint.round(2).tolist()
        data_json = {"setpoint": setpoint}
        data_byte = json.dumps(data_json).encode('UTF-8')
        print(data_byte)
        time.sleep(0.005)
        self.serial_thread.send(data_byte)

    def send_pid(self):
        pid = self.pid.round(2).tolist()
        data_json = {"pid": pid}
        data_byte = json.dumps(data_json).encode("UTF-8")
        time.sleep(0.005)
        self.serial_thread.send(data_byte)

    def send_trim(self):
        trim = self.trim.tolist()
        data_json = {"trim": trim}
        data_byte = json.dumps(data_json).encode('UTF-8')
        time.sleep(0.005)
        self.serial_thread.send(data_byte)
        return

    def send_groundEff(self):
        groundEff = self.groundEff.tolist()
        data_json = {"groundEff": groundEff}
        data_byte = json.dumps(data_json).encode('UTF-8')
        time.sleep(0.005)
        self.serial_thread.send(data_byte)
        return

    def send_arm_status(self):
        self.armed = not self.armed

        if self.armed:
            self.pushButton_arm.setText("Arm: True")
            self.pushButton_arm.setStyleSheet("background-color: rgb(153, 193, 241);""color: rgb(255, 255, 255);")
        else:
            self.pushButton_arm.setText("Arm: False")
            self.pushButton_arm.setStyleSheet("background-color: rgb(119, 118, 123);""color: rgb(0, 0, 0);")
        data_json = {"armed": self.armed}
        data_byte = json.dumps(data_json).encode('utf-8')
        print(data_byte)
        time.sleep(0.005)
        self.serial_thread.send(data_byte)

    def on_trajectory_module(self):
        if self.trajectory_on == True:
            print("Haven't finished the last trajectory")
            return
        thread_trajectory = threading.Thread(target=self.fly_on_trajectory)
        self.trajectory_on = True
        print("on fly_on_trajectory")
        thread_trajectory.start()

    def off_trajectory_module(self):
        self.trajectory_on = False

    def fly_on_trajectory(self):
        try:
            for i, trajectory_point in enumerate(self.trajectory_points):
                self.setpoint = trajectory_point
                self.send_setpoint()
                
                while True:
                    if self.trajectory_on == False:
                        return

                    if len(self.drone_pos) != 4 or np.linalg.norm(self.drone_pos[:-1]-trajectory_point[:-1])>0.08 or abs(self.drone_pos[-1]-trajectory_point[-1])>0.35:
                        time.sleep(0.005)
                    else:
                        print("last point satisfied")
                        break

        finally:
            self.trajectory_on = False
            print("exit fly_on_trajectory")


    def calibrate_yaw(self):
        if len(self.current_object_points) != 3:
            return

        position = np.mean(self.current_object_points, axis=0)

        min_diff = float('inf')
        best_pair = (0, 1)
        for i,j in combinations(range(3), 2):
            dist = np.linalg.norm(self.current_object_points[i] - self.current_object_points[j])
            diff = abs(dist - self.feature_dist)
            if diff < min_diff:
                min_diff = diff
                best_pair = (i, j)
        all_indicies = {0, 1, 2}
        feature_point_index = list(all_indicies - set(best_pair))[0]

        heading_vector = (self.current_object_points[feature_point_index] - position)[:2]
        self.yaw_trim = -np.arctan2(heading_vector[1], heading_vector[0])
        print(f"yaw_trim(rad): {self.yaw_trim}")
        print(f"yaw_trim(degree): {self.yaw_trim*57.2958}")

    def test_accuracy(self):
        errors, object_points, _ = localize_object_points(camera_poses, camera_to_world_matrix)
        if len(object_points) == 2:
            distance = np.linalg.norm(object_points[0] - object_points[1], axis=0)
            print(f"distance:{distance}\nerrors:{errors}")

    def handle_serial_response(self, data_str):
        # print(f"Sender to Py:\n{data_str}")
        return

    def show_fullscreen_frame(self, event):
        if self.last_qt_image:
            self.popup = ImagePopup(self.last_qt_image)
            self.popup.show()

    def closeEvent(self, event):
        self.serial_thread.stop()
        event.accept()

class SerialThread(QThread):
    data_received = pyqtSignal(str)

    def __init__(self, port, baudrate):
        super().__init__()
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.running = True
        time.sleep(2)

    def run(self):
        while self.running:
            if self.ser.in_waiting:
                try:
                    response = self.ser.readline().decode(errors='ignore')
                    self.data_received.emit(response)
                except Exception as e:
                    print(f"Serial read error in thread: {e}")

    def send(self, data_bytes):
        if self.ser.is_open:
            self.ser.write(data_bytes)

    def stop(self):
        self.running = False
        self.wait()    
        if self.ser.is_open:
            self.ser.close()

class ImagePopup(QMainWindow):
    def __init__(self, image):
        super().__init__()
        self.setWindowTitle("Enlarged Frame")
        label = QLabel()
        label.setPixmap(QPixmap.fromImage(image).scaled(2560, 1400, aspectRatioMode=1))
        self.setCentralWidget(label)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VideoWindow()
    window.setWindowTitle("Motion Capture and Drone Swarms")
    window.show()
    sys.exit(app.exec_())
