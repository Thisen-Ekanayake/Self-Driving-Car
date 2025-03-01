import carla
import math
import random
import time

client = carla.Client('localhost', 2000)
world = client.get_world()
blueprints = world.get_blueprint_library()
spawn_points = world.get_map().get_spawn_points()
vehicle_bp = blueprints.find('vehicle.tesla.model3')
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

for i in range(30):
    vehicle_bp = random.choice(blueprints.filter('vehicle'))