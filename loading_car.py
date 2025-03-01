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
    vehicle_bp = random.choice(blueprints.filter('vehicle*'))
    npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

for v in world.get_actors().filter('vehicle*'):
    v.set_autopilot(True)

camera_bp = blueprints.find('sensor.camera.rgb')
camera_init_trans = carla.Transform(carla.Location(z=2))
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)
camera.listen(lambda data: data.save_to_disk('out/%.6d.png' % data.frame))
#camera.stop()