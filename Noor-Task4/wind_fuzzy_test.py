from copy import copy
from matplotlib import pyplot as plt

from PythonClient import airsim
from PythonClient.multirotor.mission.fly_to_points import FlyToPoints
from PythonClient.multirotor.monitor.point_deviation_monitor import PointDeviationMonitor
import threading


def fuzzy_test_wind():
    """
    Simulates a drone flying through a series of waypoints under specific wind conditions
    and monitors the deviation from the planned path.

    Results are then plotted.
    """
    try:
        # Initialize the mission with target parameters
        mission = FlyToPoints(
            target_drone="Drone 1",
            speed=3,
            points=planned_position_list
        )
        mission.set_wind_speed(0, wind_speed, 0)  # Apply wind condition (Eastward)

        # Set up the deviation monitor
        monitor = PointDeviationMonitor(mission)

        # Run mission and monitor in parallel threads
        mission_thread = threading.Thread(target=mission.start)
        monitor_thread = threading.Thread(target=monitor.start)
        mission_thread.start()
        monitor_thread.start()

        # Wait for both threads to complete
        mission_thread.join()
        monitor_thread.join()

        # Deep copy the recorded positions for safe storage
        actual_positions = copy(monitor.est_position_array)
        all_position_array.append(actual_positions)

        # Extract coordinates for visualization
        x_coords = [point[0] for point in actual_positions]
        y_coords = [point[1] for point in actual_positions]
        z_coords = [-point[2] for point in actual_positions]  # Negate Z for proper visualization

        # Plot the actual flight path
        ax.plot(x_coords, y_coords, z_coords, label=f"Wind Speed: {wind_speed} m/s")

    except Exception as e:
        print(f"Error during fuzzy test for wind speed {wind_speed} m/s: {e}")


# Initialize data containers and test parameters
all_position_array = []
wind_speed_array = [0, 7, 14, 21]  # Different wind speeds to test
planned_position_list = [
    (0, 0, -8), (8, 0, -8), (8, 8, -8), (-8, 8, -8),
    (-8, -8, -8), (8, -8, -8), (8, 0, -8), (0, 0, -8)
]

# Set up the 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Iterate over each wind speed and run the simulation
for wind_speed in wind_speed_array:
    print(f"Running simulation for wind speed: {wind_speed} m/s")
    fuzzy_test_wind()
    airsim.MultirotorClient().reset()  # Reset simulation environment after each test

# Finalize and display the plot
ax.set_box_aspect([1, 1, 1])  # Set equal aspect ratio for axes
ax.legend()
plt.title("Drone Flight Paths Under Varying Wind Conditions")
plt.xlabel("North (+X)")
plt.ylabel("East (+Y)")
ax.set_zlabel("Altitude (+Z)")
plt.show()
