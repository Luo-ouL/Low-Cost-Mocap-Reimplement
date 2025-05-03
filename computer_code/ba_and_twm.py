# Bundle adjustment and to world matrix
from mocap.helpers import *
"""
Below code is running with all cameras,
initial camera poses(camera_poses01,02,etc) are acquired.
"""
folder_path = "extrinsic"
num_image_point_pairs = 10

## Uncover camera-params[1]
with open("mocap/camera-params_copy.json", "r") as f:
    camera_params_copy = json.load(f)
with open("mocap/camera-params.json", "r") as f:
    camera_params = json.load(f)

camera_params = camera_params_copy
with open("mocap/camera-params.json", "w") as f:
    json.dump(camera_params, f, indent=4)
print("camera-params[1] has been uncovered.")

## Acquire camera indicies
while True:
    try:
        camera_indices = input("Please input the cameras indices of bundle adjustment(all the cameras).\n")
        camera_indices = list(map(int, camera_indices.split()))
        # insure the length of the order is right
        if sorted(camera_indices) != list(range(len(camera_indices))):
            raise ValueError("Invalid input.")
        break

    except ValueError as e:
        print(f"Invalid input: {e}\nPlease enter a valid permutation.")

## Initialize and acquire 
cameras = Cameras.instance()
cv.waitKey(50)

## Read frames and collect image points
# collect image point pairs
image_point_pairs = [[] for _ in range(cameras.num_cameras)] # don't forget to convert to np.array

for _, cam_idx in enumerate(camera_indices):
    os.makedirs(f"{folder_path}/camera_{cam_idx}", exist_ok=True)

i = 0
while i < num_image_point_pairs:
    input(f"Press enter to capture number{i} images.\n")
    frames = cameras.camera_read()

    images_points = [[] for _ in range(cameras.num_cameras)]
    for j in range(cameras.num_cameras):
        images_points[j] = cameras.find_dot(frames[j])[1]
    print(f"images_points:\n{images_points}\n")

    if all(len(images_points[j]) == 1 and images_points[j][0][0] is not None for j in range(cameras.num_cameras)):
        for j in range(cameras.num_cameras):
            filename = f"{folder_path}/camera_{j}/image_{i}.jpg"
            cv.imwrite(filename, frames[j])
            image_point_pairs[j].append(images_points[j][0])
        print("Capture success, move the marker and continue.")
        i += 1
    else:
        print("Capture failed, move the marker and continue.")
        continue

image_point_pairs = np.array(image_point_pairs)
print(f"image_point_pairs:\n{image_point_pairs}\n")
correspondant_image_point_pairs = image_point_pairs.transpose(1,0,2).tolist()

if len(image_point_pairs[0]) <= 8:
    raise RuntimeError(f"image point pairs <= 8")
else:
    print("Please check and press enter to continue.:\n")

## Bundle adjustment
# acquire initial camera poses
camera_pairs = [[0, i] for i in range(1, cameras.num_cameras)]
camera_poses = [np.load(f"extrinsic/camera_poses{camera_pairs[0][0]}{camera_pairs[0][1]}.npy", allow_pickle=True)[0],
                np.load(f"extrinsic/camera_poses{camera_pairs[0][0]}{camera_pairs[0][1]}.npy", allow_pickle=True)[1]]
for camera_pair in camera_pairs[1:]:
    camera_poses.append(np.load(f"extrinsic/camera_poses{camera_pair[0]}{camera_pair[1]}.npy", allow_pickle=True)[1])
print(f"camera_poses:\n{camera_poses}.")
# bundle adjustment
camera_poses = bundle_adjustment(correspondant_image_point_pairs, camera_poses)
np.save(f"extrinsic/camera_poses.npy", camera_poses, allow_pickle=True)

print("Bundle adjustment complete")

## Establish world coordinate
while True:
    input(f"Press enter to capture image for acquiring world coordinate.(3 infrared points are needed)\n")
    frames = cameras.camera_read()
    image_point_0 = cameras.find_dot(frames[0])[1]
    image_point_1 = cameras.find_dot(frames[1])[1]
    image_point_2 = cameras.find_dot(frames[2])[1]
    if len(image_point_0) == 3 and len(image_point_1) == 3 and len(image_point_2) == 3:
        image_points = [image_point_0, image_point_1, image_point_2]
        errors, object_points, _ = find_point_correspondance_and_object_points(image_points, camera_poses, frames)
        cv.imshow("image of camera0", frames[0])
        cv.waitKey(50)
        print(f"image points of camera0:\n{image_point_0}")
        if len(object_points) != 3:
            print("Calculated object points not enough.\n")
            continue
        while len(object_points) == 3:
            usr_input = input("Choose points to be the world coordinate's origin,x_axis,y_axis\n")
            numbers = usr_input.split()
            numbers = [int(num) for num in numbers]
            if all(0 <= num <= 2 for num in numbers) and len(numbers) == 3:
                x_axis = object_points[numbers[1]] - object_points[numbers[0]]
                y_axis = object_points[numbers[2]] - object_points[numbers[0]]

                x_axis = x_axis / np.linalg.norm(x_axis)

                z_axis = np.cross(x_axis, y_axis)
                z_axis = z_axis / np.linalg.norm(z_axis)

                y_axis = np.cross(z_axis, x_axis)
                y_axis = y_axis / np.linalg.norm(y_axis)
                
                R = np.array([x_axis, y_axis, z_axis])
                t = np.dot(R,-object_points[numbers[0]]).reshape(3,1)
                camera_to_world_matrix = np.concatenate((np.concatenate((R,t),axis=1),np.array([[0,0,0,1]])),axis=0)
                np.save("extrinsic/camera_to_world_matrix.npy", camera_to_world_matrix)
                print(f"R:\n{R}\nt:\n{t}")
                break
            else:
                print("Wrong pattern!")
                continue
        break