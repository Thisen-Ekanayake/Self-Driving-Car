import carla
import pygame
import time

# Explanation
# pygame is used to capture keyboard inputs.
# The car is controlled using W, A, S, D for throttle, steering, and braking.
# The vehicle.apply_control(control) applies the control to the car in real-time.
# Press W to accelerate, S to brake, A to turn left, and D to turn right.

# Initialize pygame for keyboard control
pygame.init()

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

# Create a vehicle control object
control = carla.VehicleControl()

# Function to handle keyboard input for car control
def handle_keys():
    keys = pygame.key.get_pressed()

    # Throttle, Brake, and Steering control
    control.throttle = 0.0
    control.brake = 0.0
    control.steer = 0.0

    # Throttle (W key)
    if keys[pygame.K_w]:
        control.throttle = 1.0  # Full throttle

    # Brake (S key)
    if keys[pygame.K_s]:
        control.brake = 1.0  # Full brake

    # Steering (A and D keys)
    if keys[pygame.K_a]:
        control.steer = -1.0  # Full left steer
    if keys[pygame.K_d]:
        control.steer = 1.0  # Full right steer

    # Apply control
    vehicle.apply_control(control)

# Main loop
try:
    while True:
        # Listen to keyboard events
        handle_keys()

        # Update the world
        world.tick()

        # Sleep to control frame rate
        time.sleep(0.05)

except KeyboardInterrupt:
    pass

# Clean up
vehicle.destroy()
pygame.quit()