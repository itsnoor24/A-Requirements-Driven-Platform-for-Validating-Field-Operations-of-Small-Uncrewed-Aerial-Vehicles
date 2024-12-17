import os
import pickle
from copy import copy
from time import sleep

import numpy as np
from matplotlib import pyplot as plt

from PythonClient import airsim
from PythonClient.multirotor.mission.fly_in_circle import FlyInCircle
from PythonClient.multirotor.mission.fly_straight import FlyStraight
from PythonClient.multirotor.mission.fly_to_points import FlyToPoints
from PythonClient.multirotor.monitor.circular_deviation_monitor import CircularDeviationMonitor
from PythonClient.multirotor.monitor.drift_monitor import DriftMonitor
from PythonClient.multirotor.monitor.point_deviation_monitor import PointDeviationMonitor
import threading

# Define line styles for plotting
line_styles = ['solid', 'dashed', 'dotted']


def fuzzy_test_wind():
    """
    Simulates a drone flying in a a circular path under different wind speeds (fuzzy testing).
    It monitors deviations and logs the performance.
    """
    # Configure the mission (currently set to FlyInCircle)
    mission = FlyInCircle(
        target_drone="Drone1",
        speed=drone_speed,
        radius=8,
        altitude=8,
        iterations=1,
        center=(1, 0)
    )
    mission.set_wind_speed(0, wind_speed, 0)

    # Initialize the monitor (CircularDeviationMonitor)
    monitor = CircularDeviationMonitor(mission, 0)

    # Run mission and monitor in parallel threads
    run_threads(mission, monitor)

    # Log performance metrics
    print(f"Wind: {wind_speed} m/s, Total flight time: {mission.flight_time_in_seconds} s")
    print(f"Wind: {wind_speed} m/s, Optimal distance: {monitor.optimal_distance} m")
    print(f"Wind: {wind_speed} m/s, Actual distance: {monitor.actual_distance} m")

     # Deep copy the monitored position data
    actual_position_list = copy(monitor.est_position_array)
    all_position_array.append(actual_position_list)


def run_threads(mission, monitor):
    """
    Executes the mission/monitoring process in parallel threads .
    
    Args:
        mission: The mission object that will be executed.
        monitor: The monitor object that will observe the mission.
    """
    mission_thread = threading.Thread(target=mission.start)
    monitor_thread = threading.Thread(target=monitor.start)

    # Start both threads
    mission_thread.start()
    monitor_thread.start()

    # Wait for both threads to finish
    mission_thread.join()
    monitor_thread.join()


def graph(wind_speed_array, all_position_array, planned_position_list):
    """
    Generates a 3D plot of the drone's actual flight paths under different wind conditions.
    
    Arguments:
        wind_speed_array (list): Wind speeds used during the simulation.
         all_position_array (list): List of actual positions recorded for each wind speed.
        planned_position_list (list): List of planned waypoints.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for current_wind_speed, actual_positions in zip(wind_speed_array, all_position_array):
        # Extract coordinates for the current flight path
        x_coords = [point[0] for point in actual_positions]
        y_coords = [point[1] for point in actual_positions]
        z_coords = [-point[2] for point in actual_positions]  # Negate Z for proper visualization

        # Plot the flight path with a specific line style
        line_style = line_styles.pop(0) if line_styles else 'solid'  # Use solid if styles run out
        ax.plot(x_coords, y_coords, z_coords, linestyle=line_style,
                label=f"(Actual) Wind speed: {current_wind_speed} m/s")

    # Creating the plot appearance
    ax.set_title(f"Drone Speed = {drone_speed} m/s\nWind Speeds = {wind_speed_array} m/s (Eastward)")
    ax.set_xlabel('North (+X)')
    ax.set_ylabel('East (+Y)')
    ax.set_zlabel('Altitude (+Z)')
    ax.set_box_aspect([1, 1, 1])
    ax.legend()
    plt.show()


# Simulation parameters
drone_speed = 6  # Speed of the drone in m/s
wind_speed_array = [0, 10, 18]  # Array of wind speeds to test
planned_position_list = [(0, 0, 0), (8, 8, -8)]  # Example planned positions (not used directly)

# Load/initialize position data
if os.path.isfile('position.pkl'):
    with open('position.pkl', 'rb') as file:
        all_position_array = pickle.load(file)
else:
    all_position_array = []

    # Run the simulation for each wind speed
    for wind_speed in wind_speed_array:
        print(f"Testing wind speed: {wind_speed} m/s")
        fuzzy_test_wind()

        # Reset the simulation environment
        sleep(2)
        airsim.MultirotorClient().reset()

# Generate the graph after all simulations
graph(wind_speed_array, all_position_array, planned_position_list)

# Save the recorded positions for future use
with open('position.pkl', 'wb') as file:
    pickle.dump(all_position_array, file)
