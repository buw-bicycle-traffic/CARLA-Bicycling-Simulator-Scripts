import carla
import random

# This script connects to a CARLA server, spawns a vehicle if none are present, and performs raycasts to find the ground locations ahead and behind the vehicle.
def get_ground_locations(world, vehicle):
    vehicle_location = vehicle.get_location()
    
    # Create the starting points of the rays just above the vehicle
    start_location_front = carla.Location(vehicle_location.x + 2.0, vehicle_location.y, vehicle_location.z + 1.0)
    start_location_rear = carla.Location(vehicle_location.x - 2.0, vehicle_location.y, vehicle_location.z + 1.0)
    
    # Create the ending points of the rays far below the vehicle
    end_location_front = carla.Location(vehicle_location.x + 2.0, vehicle_location.y, vehicle_location.z - 100.0)
    end_location_rear = carla.Location(vehicle_location.x - 2.0, vehicle_location.y, vehicle_location.z - 100.0)
    
    # Perform the raycasts
    front_raycast_result = world.cast_ray(start_location_front, end_location_front)
    rear_raycast_result = world.cast_ray(start_location_rear, end_location_rear)
    
    if front_raycast_result and rear_raycast_result:
        front_hit = front_raycast_result[0]  # Whether the front ray hit something
        rear_hit = rear_raycast_result[0]    # Whether the rear ray hit something
        
        if front_hit and rear_hit:
            front_labelled_point = front_raycast_result[1]  # LabelledPoint instance for the front hit
            rear_labelled_point = rear_raycast_result[1]    # LabelledPoint instance for the rear hit
            
            front_location = front_labelled_point.location
            rear_location = rear_labelled_point.location
            
            return front_location, rear_location
    return None, None

if __name__ == "__main__":
    # Connect to the Carla server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    
    # Get a list of vehicle actors
    vehicles = world.get_actors().filter('vehicle.*')
    
    if vehicles:
        vehicle = vehicles[0]
    else:
        # If no vehicles are found, spawn a new vehicle
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find('vehicle.tesla.model3')  # Choose a vehicle blueprint
        spawn_point =random.choice(world.get_map().get_spawn_points())  # Choose a spawn point
        print(spawn_point)
        
        # Spawn the vehicle
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)

    # Get the ground locations ahead and behind the vehicle
    front_location, rear_location = get_ground_locations(world, vehicle)
    
    if front_location and rear_location:
        # Calculate gradient using the difference in z-coordinates
        gradient = (front_location.z - rear_location.z) / 4.0  # 4.0 is the distance between the front and rear ray lines
        
        print(f"Gradient: {gradient}")
    else:
        print("Ground locations not found.")
