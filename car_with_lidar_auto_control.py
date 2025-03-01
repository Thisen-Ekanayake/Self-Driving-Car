import carla
import numpy as np
import open3d as o3d
import cv2
import time


# What This Script Does

# Spawns a Tesla Model 3 at a random location.
# Attaches a LIDAR sensor (captures 3D point clouds).
# Attaches a camera (captures RGB images).
# Drives forward for 5 seconds then stops.
# Visualizes the LIDAR and camera data in real-time.
# Resets by destroying the actors (clean exit).


# Connect to CARLA server
client = carla.Client("localhost", 2000)
client.set_timeout(10.0)

# Load world & get blueprint library
world = client.get_world()
blueprints = world.get_blueprint_library()

# Spawn a vehicle
vehicle_bp = blueprints.filter("model3")[0]  # Tesla Model 3
spawn_point = world.get_map().get_spawn_points()[0]
vehicle = world.spawn_actor(vehicle_bp, spawn_point)

# Attach a LIDAR sensor
lidar_bp = blueprints.find("sensor.lidar.ray_cast")
lidar_bp.set_attribute("range", "50")  # 50m range
lidar_bp.set_attribute("rotation_frequency", "10")
lidar_bp.set_attribute("channels", "32")
lidar_bp.set_attribute("points_per_second", "50000")

lidar_transform = carla.Transform(carla.Location(x=0, y=0, z=2))
lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)

# Attach a camera sensor
camera_bp = blueprints.find("sensor.camera.rgb")
camera_bp.set_attribute("image_size_x", "800")
camera_bp.set_attribute("image_size_y", "600")
camera_bp.set_attribute("fov", "90")

camera_transform = carla.Transform(carla.Location(x=1.5, y=0, z=2))
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

# Callback function to process LIDAR data
def process_lidar(data):
    points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (len(points) // 4, 4))[:, :3]  # (x, y, z) points
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([pcd])

# Callback function to process camera images
def process_camera(image):
    img = np.reshape(np.array(image.raw_data), (image.height, image.width, 4))[:, :, :3]
    cv2.imshow("Camera", img)
    cv2.waitKey(1)

# Listen for sensor data
lidar.listen(lambda data: process_lidar(data))
camera.listen(lambda image: process_camera(image))

# Manual vehicle control (Drive forward for 5 seconds)
control = carla.VehicleControl(throttle=0.5, steer=0.0)
vehicle.apply_control(control)
time.sleep(5)

# Stop the vehicle
control = carla.VehicleControl(throttle=0.0, brake=1.0)
vehicle.apply_control(control)

# Destroy actors & reset
lidar.destroy()
camera.destroy()
vehicle.destroy()
cv2.destroyAllWindows()