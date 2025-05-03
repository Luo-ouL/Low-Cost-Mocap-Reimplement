from mocap.helpers import *
"""
Below code is running with 2 cameras,
make sure only 2 cameras are chosen,
doing this in cameras pairs 0&1, 0&2, 0&3....
"""

folder_path = "extrinsic"

num_image_point_pairs = 10
actual_distance = 0.1024

"""
If you have 3 cameras,
enter 0 1 for the first run,
and 0 2 for the second run
to calibrate the extrinsic parameters between cameras step by step.
"""
while True:
    try:
        camera_indices = input("Please enter the cameras' indices(0 1 for camera0,1 for example).\n")
        camera_indices = list(map(int, camera_indices.split()))
        # insure the length of the order is right
        if len(camera_indices) != 2 or camera_indices[0] != 0:
            raise ValueError("Invalid input.")
        break
    except ValueError as e:
        print(f"Invalid input: {e}\nPlease enter a valid permutation.")

## Cover camera-params[1]
with open("mocap/camera-params_copy.json", "r") as f:
    camera_params_copy = json.load(f)
with open("mocap/camera-params.json", "r") as f:
    camera_params = json.load(f)

camera_params = camera_params_copy
camera_params[1] = camera_params_copy[camera_indices[1]]
with open("mocap/camera-params.json", "w") as f:
    json.dump(camera_params, f, indent=4)
print("camera-params[1] is now covered.")

## Read frames and collect image points
cameras = Cameras.instance()
cv.waitKey(50)

image_point_pairs = [[],[]]
for _, cam_idx in enumerate(camera_indices):
    os.makedirs(f"{folder_path}/camera_{cam_idx}", exist_ok=True)

i = 0
while i < num_image_point_pairs:
    input(f"Press enter to capture number{i} image\n")
    for _ in range(1):
        frames = cameras.camera_read()
    frames = cameras.camera_read()
    image_point_0 = cameras.find_dot(frames[0])[1]
    image_point_1 = cameras.find_dot(frames[1])[1]
    print(image_point_0, image_point_1)
    if len(image_point_0)==1 and len(image_point_1)==1 and image_point_0[0][0] is not None and image_point_1[0][0] is not None:
        image_point_pairs[0].append(image_point_0[0])
        image_point_pairs[1].append(image_point_1[0])
        filename0 = f"{folder_path}/camera_{camera_indices[0]}/image_{i}.jpg"
        filename1 = f"{folder_path}/camera_{camera_indices[1]}/image_{i}.jpg"
        cv.imwrite(filename0, frames[0])
        cv.imwrite(filename1, frames[1])
        print("Capture successfully, move the marker and continue.")
        i += 1
    else:
        filename0 = f"{folder_path}/camera_{camera_indices[0]}/image_{i}.jpg"
        filename1 = f"{folder_path}/camera_{camera_indices[1]}/image_{i}.jpg"
        cv.imwrite(filename0, frames[0])
        cv.imwrite(filename1, frames[1])
        print("Capture failed, move the marker and continue.")
        continue

image_point_pairs = np.array(image_point_pairs)
print(f"image_point_pairs:\n{image_point_pairs}\n")

if len(image_point_pairs[0]) <= 8:
    raise RuntimeError(f"image point pairs <= 8")
else:
    print("Please check and press enter to continue.:\n")

## Calculate F,E,R,t
F, _ = cv.findFundamentalMat(image_point_pairs[0], image_point_pairs[1], cv.FM_RANSAC, 1, 0.99999)
print(f"Fundamental matrix:\n{F}")

K1 = cameras.get_camera_params(0)["intrinsic_matrix"]
K2 = cameras.get_camera_params(1)["intrinsic_matrix"]
print(f"Intrinsic matrix K1:\n{K1}")
print(f"Intrinsic matrix K2:\n{K2}")

E = essential_from_fundamental(F, K1, K2)
print(f'Essential matrix:\n{E}')

Rs, ts = motion_from_essential(E)
for i in range(4):
    print(f'Rotations_matrices{i}:\n{Rs[i]}\ntranslations{i}:\n{ts[i]}')
print()

## Choose the right combination of R,t
correspondant_image_point_pairs = image_point_pairs.transpose(1,0,2).tolist()
print(f"correspondant image point pairs:\n{correspondant_image_point_pairs}\n")
positive_percentage = []
for index in range(4):
    camera_poses = [
        {
            "R": np.eye(3),
            "t": np.array([0, 0, 0], dtype = np.float32)
        },
        {
            "R": Rs[index],
            "t": ts[index]
        }
    ]
    object_points = triangulate_points(correspondant_image_point_pairs, camera_poses)
    positive_count = (object_points[:,2] > 0).sum()
    positive_percentage.append(positive_count/len(object_points))
    print(f"index{index} positive percentage: {positive_count/len(object_points):.1%}")
index = positive_percentage.index(max(positive_percentage))
print(f"index{index} is chosen\n")

## Bundle adjustment
camera_poses = [
    {
        "R": np.eye(3),
        "t": np.array([0, 0, 0], dtype = np.float32)
    },
    {
        "R": Rs[index],
        "t": ts[index]
    }
]

camera_poses = bundle_adjustment(correspondant_image_point_pairs, camera_poses)
input("Bundle adjustment complete, please check and press enter to continue.\n")

## Determine the scale of t
num_scale_coef = 10
scale_coef = []
i = 0
while i < num_scale_coef:
    input(f"Press enter to capture image{i} for the determination of t(2 infrared points are needed).\n")
    for _ in range(1):
        frames = cameras.camera_read()
    frames = cameras.camera_read()
    image_point_0 = cameras.find_dot(frames[0])[1]
    image_point_1 = cameras.find_dot(frames[1])[1]
    if len(image_point_0) == 2 and len(image_point_1) == 2:
        image_points = [image_point_0, image_point_1]
        errors, object_points, _ = find_point_correspondance_and_object_points(image_points, camera_poses, frames)
        if len(object_points) == 2:
            mea_distance = np.sqrt(np.sum((object_points[0]-object_points[1])**2))
            scale_coef.append(actual_distance/mea_distance)
            print(f"scale_coef[{i}]:\n{scale_coef[i]}")
            i += 1
        else:
            print("Caculated object points <= 2\n")
            continue
    else:
        print("image points num wrong.\n")
        continue

mean = np.mean(scale_coef)
std = np.std(scale_coef)
scale_coef_average = np.mean([x for x in scale_coef if abs(x - mean) < 2 * std]) # 2σ rule
print("Final scale coef:\n", scale_coef_average)

camera_poses[1]["t"] = camera_poses[1]["t"] * scale_coef_average
np.save(f"extrinsic/camera_poses{camera_indices[0]}{camera_indices[1]}.npy", camera_poses, allow_pickle=True)

## Uncover camera-params[1]
camera_params = camera_params_copy
with open("mocap/camera-params.json", "w") as f:
    json.dump(camera_params, f, indent=4)
print("camera-params[1] has been uncovered.")
