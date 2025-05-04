# First of all
&nbsp;&nbsp;&nbsp;&nbsp;This is a reimplement of [Joshua Bird's repository](https://github.com/jyjblrd/Low-Cost-Mocap)  
&nbsp;&nbsp;&nbsp;&nbsp;It might be considered an easier version, since the React frontend has been replaced with a PyQt frontend, and the no-cv-sfm branch is used.  
&nbsp;&nbsp;&nbsp;&nbsp;There is a [YouTube video](https://youtu.be/QF4qJRMAZwg) demonstrating the drone's hover and the use of the PyQtFrontend.

# What's changed?
(1) I build a PyQtFrontend instead of using ReactFrontend.
![PyQtFrontend](pictures/PyQtFrontend.png)
(2) I separated the [intrinsic](computer_code/acquire_intrinsic_params.py) and [extrinsic](computer_code/acquire_extrinsic_params.py) parameter acqusition into independent Python programs.  
(3) I rewrote the [point correlation algorithm](computer_code/mocap/helpers.py#L323-L408).  
(4) I rewrote the [receiver.cpp L43-44](esp32_receiver/src/receiver.cpp#L43-L44)[receiver.cpp L193-198](esp32_receiver/src/receiver.cpp#L195-L198) so that both hovering and trajectory tracking can have a specified heading/yaw value. The PID output is in world coordinate, so we need to project it into the drone's local coordinate system, which corresponds to the SBUS values.  
(5) I customized the [Arduino PID library code .cpp](esp32_receiver/src/PID_v1_bc.cpp#L47-L139)and [Arduino PID library code .h](esp32_receiver/src/PID_v1_bc.h), in this way, if yaw = -π + 0.05 and yawSetpoint = π - 0.05, the drone can intelligently choose the shorter path (−0.1) instead of the longer one (2π − 0.1).  
(6) I used ![webcam](pictures/webcam.jpeg) instead of ps3-eye camera. 3 cameras are used(theoretically, 4 or more cameras are also feasible).  
(7) I used the [no-cv-sfm branch](https://github.com/jyjblrd/Low-Cost-Mocap/tree/no-cv-sfm)(actually the [3 alternative functions](computer_code/mocap/helpers.py#L508-L571)) to avoid the hassle of compiling cv-sfm.  
(8) There are many minor changes that are hard to explain, as I’ve almost reconstructed the original project. However, the core functionality remains largely unchanged — or rather, the math behind the motion capture is still the same.

# How to run the code?
(0) I have 3 cameras: camera0 (ID 4), camera1 (ID 6), and camera2 (ID 8). The numbers 0, 1, and 2 are user-defined labels, while 4, 6, and 8 are the device indices used in OpenCV (e.g., camera = cv.VideoCapture(4)), which may vary depending on the system.  
(1) Run [acquire_intrinsic_params.py](computer_code/acquire_intrinsic_params.py) to obtain the intrinsic parameters, and fill them into [camera-params_copy.json](computer_code/mocap/camera-params_copy.json). This needs to be done for each camera individually.  
(2) Run [acquire_extrinsic_params.py](computer_code/acquire_extrinsic_params.py) to obtain the extrinsic parameters for camera pairs (0&1, 0&2). The files [camera_poses01.npy](computer_code/extrinsic/camera_poses01.npy) and [camera_poses02.npy](computer_code/extrinsic/camera_poses02.npy) will be generated in the extrinsic directory. First, move a single infrared marker ![single infrared marker](pictures/1_marker_board.jpeg) and capture 10 images from each of the two cameras to obtain preliminary camera poses. Then, move two infrared markers ![two infrared markers](pictures/2_marker_board.jpeg) (where actual_distance refers to the known distance between them, e.g., 0.1024 m) and capture another 10 images to determine the scale. The [number of images](computer_code/acquire_extrinsic_params.py#L10) and the [actual_distance](computer_code/acquire_extrinsic_params.py#L11) must be specified.  
(3) Run [ba_and_twm.py](computer_code/ba_and_twm.py) to perform bundle adjustment and obtain the [camera_poses(3cameras)](computer_code/extrinsic/camera_poses.npy) and [camera_to_world_matrix.npy](computer_code/extrinsic/camera_to_world_matrix.npy) (all cameras should be connected at this stage). Then, you can run [frontend.py](computer_code/frontend.py) to test pure localization without the drone (you'll need to connect the computer to an ESP32). Make sure to specify the serial port in the [SERIAL_PORT](computer_code/frontend.py#L55) variable, e.g., /dev/ttyACM0. Similarly, capture 10 images for this step. Finally, capture one image with 3 markers to determine the to_world_matrix. In my setup, I use this board with 3 markers ![this board](pictures/3_markers_board.jpeg) and choose one point as the origin, with the other two points representing the directions of the x- and y-axes from camera0's perspective: ![camera0's perspective](pictures/3markers_in_camera0_view.png).  
(4) Flash the [esp32_sender](esp32_sender/src/sender.cpp) for the computerside and the [esp32_receiver](esp32_receiver/src/receiver.cpp) for the droneside.  
(5) Run [frontend.py](computer_code/frontend.py) to use the UI and control the drone.

# How to use the UI?
![PyQtFrontend](pictures/PyQtFrontend.png)
(1) The upper part of the UI displays the video streams from the three cameras. The lower left shows the positioning results in the world coordinate frame — a drone lying on the ground (x-y plane), with a green line indicating its heading. The remaining area is the control module.  
(2) You can see there are many pushButton and lineEdit widgets. We need to fill in the required values and click the corresponding buttons; the frontend will then send the data (such as setpoints, trim values, ground effect compensation, PID parameters, and trajectory) to the drone, as shown in the YouTube video.  
&nbsp;&nbsp;&nbsp;&nbsp;To make the drone hover, we first need to calibrate the yaw, since the drone does not know its initial heading. Then, we must check the locate_drones_on option so that positioning data is sent to the drone. After that, we send the setpoint to the drone, and finally, click the Arm button to arm the drone.  
&nbsp;&nbsp;&nbsp;&nbsp;To use the trajectory tracking module, for example, we input waypoints in the format {x, y, z, yaw}, like this: 0 0 0.2 0\n0.3 0 0.2 0.6\n0.3 0.3 0.2 1.2. Then, we specify the expected velocity or the spacing between consecutive points, e.g., 0.1. Based on that, a sequence of intermediate setpoints with fixed spacing will be generated. Finally, we click the trajectory_on button to start trajectory tracking — meaning the setpoints will be sent one by one, each after the previous one is reached.  
&nbsp;&nbsp;&nbsp;&nbsp;By the way, the Test Accuracy button might be a bit confusing: when you click it and exactly two points are detected, the terminal will output the distance between them. You can use this to evaluate the positioning accuracy by comparing the measured distance with the actual physical distance.

# What is the reimplement's flaw?
(1) Locate about more than 6(approximately) points is unstable, probably due to the point correlation algorithm or the camera calibration's bad accuracy.![9 points situation for example](pictures/9points.png)
&nbsp;&nbsp;&nbsp;&nbsp;In fact, when there are only a few points, the positioning accuracy can reach millimeter-level. However, when there are many points, mismatches tend to occur—since we need to match image points with epipolar lines—leading to sudden jumps due to ambiguous matches. This instability can be fatal for drone localization.  
(2) It cannot locate and control two or more drones at the same time. In fact, controlling two drones is possible by adding a Kalman Filter (which essentially uses the previous state to predict the next one, and then uses the predicted state to help resolve multiple drone matches).  
&nbsp;&nbsp;&nbsp;&nbsp;However, locating three or more drones to build a swarm is not feasible due to the limited positioning accuracy of this reimplementation. Therefore, I decided not to implement the Kalman Filter or support multiple drones.

# Finally
The origin project is really amazing and I've learned so much from it.  
Please forgive any bugs in the code as I’m still improving my programming skills.
