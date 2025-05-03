## This code is based on https://github.com/jyjblrd/Low-Cost-Mocap/discussions/11
import cv2 as cv
import os
import shutil
import numpy as np
import glob

folder_name = "intrinsic"

## Delete cache
shutil.rmtree(folder_name, ignore_errors=True)
os.makedirs(folder_name, exist_ok=True)

## List all the cameras' index to check
def check_camera_devices():
    for index in range(15):
        cap = cv.VideoCapture(index)
        if cap.isOpened():
            print(f"Camera {index} is available.")
            cap.release()
        else:
            print(f"Camera {index} is NOT available.\n")
    return None
check_camera_devices()

## Choose the webcam and initialize
while True:
    try:
        camera_index = int(input("Please enter your camera's indices (the computer may recognize the cameras as 4, 6, etc., depending on the system).\n"))
        break
    except ValueError:
        print("Invalid input, please enter a number.\n")
        continue
camera = cv.VideoCapture(camera_index)
if not camera.isOpened():
    print("Error: Could not open webcam.\n")
    exit()

camera.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))
camera.set(cv.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
camera.set(cv.CAP_PROP_FPS, 120)
CAP_PROP_FRAME_WIDTH = camera.get(cv.CAP_PROP_FRAME_WIDTH)
CAP_PROP_FRAME_HEIGHT = camera.get(cv.CAP_PROP_FRAME_HEIGHT)
CAP_PROP_FPS = camera.get(cv.CAP_PROP_FPS)
print(f"Real width: {CAP_PROP_FRAME_WIDTH}")
print(f"Real height: {CAP_PROP_FRAME_HEIGHT}")
print(f"Real fps: {CAP_PROP_FPS}")
input("Press enter to continue.\n")

## Create directory to save images
os.makedirs(f'{folder_name}/camera_{camera_index}', exist_ok=True)

## Capture 10 images
num = 0
while num < 10:
    input("Press enter to start capture.\n")
    for _ in range(5): # ensure frame is newest
        camera.read()
    ret, frame = camera.read()
    filename = f'{folder_name}/camera_{camera_index}/image_{num}.jpg'
    cv.imshow(f"{filename}", frame)
    cv.waitKey(1000)
    cv.destroyAllWindows()
    while True:
        try:
            usr_input = int(input("Input 1 to save image, other number to not save:\n"))
            break
        except ValueError:
            print("Invalid input, please enter a number.\n")
            continue
    if usr_input == 1:
        cv.imwrite(filename, frame)
        print(f"image{num} captured")
        num += 1
    else:
        continue

## Calculate intrinsic matrix and distortion coef
image_calibrated = f'{folder_name}/camera_{camera_index}_c'
os.makedirs(image_calibrated, exist_ok=True)

CHECKERBOARD = (6,5)
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objpoints = []
imgpoints = []

objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None

# Extracting path of individual image stored in a given directory
images = glob.glob(f'{folder_name}/camera_{camera_index}/*.jpg')
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    # If desired number of corners are found in the image then ret = true
    # ret, corners = cv.findChessboardCorners(gray, CHECKERBOARD, cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK + cv.CALIB_CB_NORMALIZE_IMAGE)
    ret, corners = cv.findChessboardCorners(gray, CHECKERBOARD, cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE)

    """
    If desired number of corner are detected,
    we refine the pixel coordinates and display 
    them on the images of checker board
    """
    if ret == True:
        objpoints.append(objp.copy())
        corners2 = cv.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
        imgpoints.append(corners2)
        img = cv.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)

    if ret == True:
        print(f"{fname}: Corners detected success")
    else:
        print(f"{fname}: Corners detected failed")

    new_frame_name = image_calibrated + '/' + os.path.basename(fname)
    cv.imwrite(new_frame_name, img)

h,w = img.shape[:2]

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera matrix:\n")
print(mtx)
print("dist:\n")
print(dist)

# Clean up
camera.release()
cv.destroyAllWindows() # Close any OpenCV windows if used