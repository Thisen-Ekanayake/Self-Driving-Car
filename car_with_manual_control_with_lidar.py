import carla
import pygame
import time
import numpy as np
import open3d as o3d

# Initialize pygame
pygame.init()

# Connect to CARLA server
client = carla.Client("localhost", 2000)
client.set_timeout(10.0)

# Load world
world = client.get_world()
blueprints = world.get_blueprint_library()

# Remove existing vehicles to avoid collision issues
for actor in world.get_actors():
    if 'vehicle' in actor.type_id:
        actor.destroy()

# Get a spawn point
spawn_points = world.get_map().get_spawn_points()
spawn_point = spawn_points[0]

# Spawn vehicle
vehicle_bp = blueprints.filter("model3")[0]
vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
if vehicle is None:
    raise RuntimeError("Failed to spawn vehicle!")

# Attach LIDAR sensor
lidar_bp = blueprints.find("sensor.lidar.ray_cast")
lidar_bp.set_attribute("range", "50")  
lidar_bp.set_attribute("rotation_frequency", "10")
lidar_bp.set_attribute("channels", "32")
lidar_bp.set_attribute("points_per_second", "50000")

lidar_transform = carla.Transform(carla.Location(x=0, y=0, z=2))
lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)

# Create vehicle control object
control = carla.VehicleControl()

# Initialize Open3D visualizer
vis = o3d.visualization.Visualizer()
vis.create_window()
pcd = o3d.geometry.PointCloud()
vis.add_geometry(pcd)

# Pygame screen for user input
screen = pygame.display.set_mode((400, 300))

# Global storage for LIDAR data
lidar_data = np.array([])

# Callback function for LIDAR
def process_lidar(data):
    global lidar_data
    points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (len(points) // 4, 4))[:, :3]  
    lidar_data = points

# Start listening to LIDAR
lidar.listen(lambda data: process_lidar(data))

# Function to control vehicle
def handle_keys():
    keys = pygame.key.get_pressed()
    control.throttle = 0.0
    control.brake = 0.0
    control.steer = 0.0

    if keys[pygame.K_w]: control.throttle = 1.0
    if keys[pygame.K_s]: control.brake = 1.0
    if keys[pygame.K_a]: control.steer = -1.0
    if keys[pygame.K_d]: control.steer = 1.0

    vehicle.apply_control(control)

# Wait until LIDAR receives data
while len(lidar_data) == 0:
    print("Waiting for LIDAR data...")
    time.sleep(1)

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Handle car control
    handle_keys()

    # Update LIDAR visualization (only if data exists)
    if len(lidar_data) > 0:
        pcd.points = o3d.utility.Vector3dVector(lidar_data)
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

    # Limit loop speed
    time.sleep(0.05)

# Cleanup
lidar.destroy()
vehicle.destroy()
pygame.quit()
vis.destroy_window()
