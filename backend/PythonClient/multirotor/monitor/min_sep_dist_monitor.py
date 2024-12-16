import time
import numpy as np
from drone_monitor_base import GlobalMonitor  # simplified import


class DroneDistanceMonitor(GlobalMonitor):
    """
    My custom monitor to check if drones are keeping safe distances from each other
    """

    def __init__(self, min_horizontal_dist=1, min_vertical_dist=0):
        super().__init__()
        # Basic settings
        self.is_running = True
        self.had_violation = False
        self.update_interval = 1  # seconds between checks

        # Distance requirements
        self.min_horizontal_dist = min_horizontal_dist
        self.min_vertical_dist = min_vertical_dist

        # Tracking data
        self.drone_list = []
        self.num_drones = len(self.all_drone_names)
        self.start_timestamp = None

        # Arrays for distance calculations
        self.vertical_distances = []
        self.horizontal_distances = []
        self.drone_positions = []

    def start_monitoring(self):
        # Log monitor startup
        msg = (
            f"Starting distance monitor for drones: {self.all_drone_names}\n"
            f"Minimum distances: {self.min_horizontal_dist}m horizontal, "
            f"{self.min_vertical_dist}m vertical\n"
            f"Checking every {self.update_interval} seconds"
        )
        self.log_info(msg)

        # Setup
        self.start_timestamp = time.time()
        self._setup_drone_list()

        # Main monitoring loop
        while self.is_running:
            self._update_distances()
            self._check_distance_violations()
            time.sleep(self.update_interval)

        # Final report
        self._create_final_report()

    def stop_monitoring(self):
        self.is_running = False

    def _setup_drone_list(self):
        self.drone_list = [name for name in self.all_drone_names]

    def _update_distances(self):
        # Get current positions of all drones
        self.drone_positions = [
            self.client.simGetObjectPose(drone).position
            for drone in self.all_drone_names
        ]

        # Calculate distances between all drones
        self._calc_horizontal_distances()
        self._calc_vertical_distances()

    def _calc_horizontal_distances(self):
        distances = []
        for i in range(self.num_drones):
            row = []
            for j in range(self.num_drones):
                dx = self.drone_positions[i].x_val - self.drone_positions[j].x_val
                dy = self.drone_positions[i].y_val - self.drone_positions[j].y_val
                dist = np.sqrt(dx**2 + dy**2)
                row.append(dist)
            distances.append(row)
        self.horizontal_distances = np.array(distances)

    def _calc_vertical_distances(self):
        distances = []
        for i in range(self.num_drones):
            row = []
            for j in range(self.num_drones):
                dist = abs(
                    self.drone_positions[i].z_val - self.drone_positions[j].z_val
                )
                row.append(dist)
            distances.append(row)
        self.vertical_distances = np.array(distances)

    def _check_distance_violations(self):
        if self.min_horizontal_dist > 0:
            self._check_horizontal_violations()

    def _check_horizontal_violations(self):
        violations = np.where(self.horizontal_distances < self.min_horizontal_dist)

        for i in range(len(violations[0])):
            drone1_idx = violations[0][i]
            drone2_idx = violations[1][i]

            # Skip self-comparisons
            if drone1_idx != drone2_idx:
                self.had_violation = True
                current_time = round(time.time() - self.start_timestamp)
                current_dist = round(
                    self.horizontal_distances[drone1_idx][drone2_idx], 2
                )

                msg = (
                    f"Distance violation between {self.drone_list[drone1_idx]} and "
                    f"{self.drone_list[drone2_idx]} at {current_time}s - "
                    f"Distance: {current_dist}m"
                )
                self.log_error(msg)

    def _create_final_report(self):
        if self.had_violation:
            self.log_error("Mission completed - Distance violations detected")
        else:
            self.log_info("Mission completed - All distances maintained")
        self.save_report()
