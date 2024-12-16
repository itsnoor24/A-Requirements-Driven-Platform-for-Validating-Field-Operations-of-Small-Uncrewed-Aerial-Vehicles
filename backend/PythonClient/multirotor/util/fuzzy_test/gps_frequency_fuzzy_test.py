import os
import pickle
import random
import threading
from copy import copy
from matplotlib import pyplot as plt
from PythonClient import airsim
from PythonClient.multirotor.mission.fly_in_circle import FlyInCircle
from PythonClient.multirotor.monitor.circular_deviation_monitor import (
    CircularDeviationMonitor,
)


# Function to run mission and monitor in separate threads
def run_threads(mission, monitor):
    mission_thread = threading.Thread(target=mission.start)
    monitor_thread = threading.Thread(target=monitor.start)
    mission_thread.start()
    monitor_thread.start()
    mission_thread.join()
    monitor_thread.join()


# Fuzz test for GPS frequency and various environmental conditions
def fuzzy_test_gps_frequency(hz, drone_speed, wind_speed, center, radius):
    airsim.MultirotorClient().reset()  # Reset simulation for each test
    mission = FlyInCircle(
        target_drone="Drone 1",
        speed=drone_speed,
        radius=radius,
        altitude=20,
        iterations=1,
        center=center,
    )
    mission.set_wind_speed(
        wind_speed[0], wind_speed[1], wind_speed[2]
    )  # Random wind direction and speed
    monitor = CircularDeviationMonitor(mission, deviation_percentage=0)
    monitor.dt = 1 / hz  # Set monitor time step according to GPS frequency
    run_threads(mission, monitor)
    print(f"Testing GPS frequency: {hz} Hz")
    print(f"Total flight time: {mission.flight_time_in_seconds} s")
    print(f"Optimal distance: {monitor.optimal_distance} m")
    print(f"Actual distance: {monitor.actual_distance} m")
    actual_position_list = copy(monitor.est_position_array)
    all_position_array.append(actual_position_list)


# Function to visualize results in 3D
def graph(gps_hz, all_position_array):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    for cur_gps_hz, actual_position_list in zip(gps_hz, all_position_array):
        x2 = [point[0] for point in actual_position_list]
        y2 = [point[1] for point in actual_position_list]
        z2 = [-point[2] for point in actual_position_list]
        ax.plot(x2, y2, z2, label=f"(Actual) GPS frequency: {cur_gps_hz} Hz")

    ax.set_title(
        "GPS Frequency Fuzzy Test\nDrone Speed: 4 m/s\nWind Speed: 5 m/s East (+Y)"
    )
    ax.set_xlabel("X axis")
    ax.set_ylabel("Y axis")
    ax.set_zlabel("Z axis")
    ax.legend()
    plt.show()


# Function to generate random test parameters for fuzz testing
def generate_fuzzed_parameters():
    # Fuzzing GPS frequency (Hz), drone speed (m/s), wind speed, and circle parameters
    gps_hz = random.choice([5, 10, 15, 30, 45, 60])  # Fuzzed GPS frequencies
    drone_speed = random.uniform(3, 8)  # Fuzzed drone speed between 3 and 8 m/s
    wind_speed = [
        random.uniform(-5, 5),
        random.uniform(-5, 5),
        random.uniform(0, 10),
    ]  # Random wind vector
    center = [random.uniform(-10, 10), random.uniform(-10, 10)]  # Random circle center
    radius = random.uniform(5, 15)  # Random radius for circular flight
    return gps_hz, drone_speed, wind_speed, center, radius


# Main function
if __name__ == "__main__":
    if os.path.isfile("gps.pkl"):
        all_position_array = pickle.load(open("gps.pkl", "rb"))
    else:
        all_position_array = []

        num_tests = 10

        for _ in range(num_tests):
            gps_hz, drone_speed, wind_speed, center, radius = (
                generate_fuzzed_parameters()
            )
            print(
                f"Running fuzz test with GPS frequency: {gps_hz} Hz, Drone speed: {drone_speed} m/s"
            )
            fuzzy_test_gps_frequency(gps_hz, drone_speed, wind_speed, center, radius)

        # Reset the simulation for each run (done within the fuzz test)

    # Visualize the results in a 3D graph
    gps_hz_values = [
        random.choice([5, 15, 45]) for _ in range(len(all_position_array))
    ]  # Mock list of GPS Hz for graph
    graph(gps_hz_values, all_position_array)

    # Save results to file
    pickle.dump(all_position_array, open("gps.pkl", "wb"))
