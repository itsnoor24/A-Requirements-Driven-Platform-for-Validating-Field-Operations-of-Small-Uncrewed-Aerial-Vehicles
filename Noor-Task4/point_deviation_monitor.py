from time import sleep
from PythonClient.multirotor.util.geo.geo_util import GeoUtil
from PythonClient.multirotor.util.graph.three_dimensional_grapher import ThreeDimensionalGrapher
from PythonClient.multirotor.monitor.abstract.single_drone_mission_monitor import SingleDroneMissionMonitor


class PointDeviationMonitor(SingleDroneMissionMonitor):
    """
    Monitors a drone mission to make sure that the drone stays within a specified deviation percentage
    of the planned path. Logs the breaches and generates 3D graphs of the actual vs. planned path.
    """

    def __init__(self, mission, deviation_percentage=15):
        """
        Initialize the PointDeviationMonitor.

        Arguments:
            mission: The drone mission object containing planned waypoints.
            deviation_percentage (float): Maximum allowable deviation percentage from the planned path.
        """
        super().__init__(mission)
        self.deviation_percentage = deviation_percentage
        self.mission = mission
        self.target_drone = mission.target_drone

        # Internal tracking variables
        self.breach_flag = False
        self.passed = False
        self.dt = 0.02  # Update interval in seconds (50 Hz)
        self.point_queue = None

        # Position and distance tracking
        self.est_position_array = []
        self.obj_position_array = []
        self.optimal_distance = None
        self.actual_distance = None
        self.actual_deviation_percentage = None

    def start(self):
        """
        Starts the monitoring process, logging the mission and updating the drone's position.
        """
        # Ensure the mission type is compatible with the monitor
        if type(self.mission).__name__ not in self.polygon_mission_names:
            return

        # Initialize waypoint queue with home point included
        self.point_queue = self.mission.points.copy()
        self.point_queue.insert(0, [0, 0, 0])  # Add home point at the beginning

        self.append_info_to_log(
            f"{self.target_drone}; Speed: {self.mission.speed} m/s with wind: {self.wind_speed_text}"
        )

        # Calculate the optimal (planned) path distance
        self.optimal_distance = self.calculate_planned_distance()

        # Begin position monitoring
        self.update_position()

        # Finalize and stop monitoring
        self.stop()

    def stop(self):
        """
        Finalizes the monitoring process and logging results and generating graphs.
        """
        self.append_info_to_log(f"{self.target_drone}; Mission task finished")

        # Calculate the actual traveled distance
        self.actual_distance = self.calculate_actual_distance()

        if self.breach_flag:
            self.append_fail_to_log(
                f"{self.target_drone}; Deviation Violation (>{self.deviation_percentage} meters)"
            )
        else:
            self.append_pass_to_log(
                f"{self.target_drone}; No deviation violation (<{self.deviation_percentage} meters)"
            )

        # Check if the actual distance is valid
        if not self.actual_distance or self.actual_distance == 0:
            self.append_fail_to_log(f"{self.target_drone}; Actual distance is 0")
        else:
            # Calculate deviation percentage
            self.actual_deviation_percentage = (
                (self.actual_distance - self.optimal_distance) / self.optimal_distance
            )

        # Generate 3D graph of planned vs. actual paths
        self.draw_trace_3d()

        # Save the monitoring report
        self.save_report()

    def calculate_planned_distance(self):
        """
        Calculates the total distance of the planned path from the drone's initial position.

        Returns:
            float: The total planned path distance.
        """
        original_position = self.client.getMultirotorState(self.target_drone).kinematics_estimated.position
        distance = self.get_distance_btw_points(
            [original_position.x_val, original_position.y_val, original_position.z_val],
            self.mission.points[0]
        )

        # Sum up distances between consecutive waypoints
        for i in range(1, len(self.mission.points)):
            distance += self.get_distance_btw_points(self.mission.points[i - 1], self.mission.points[i])

        return distance

    def update_position(self):
        """
        Continuously updates the drone's estimated position during the mission.

        Logs breaches when the drone deviates from the planned path beyond the allowed threshold.
        """
        self.append_info_to_log(
            f"{self.target_drone}; Registering drone location every {self.dt} seconds"
        )

        while self.mission.state != self.mission.State.END:
            # Retrieve the estimated and object positions
            estimated_position = self.client.getMultirotorState(
                vehicle_name=self.target_drone
            ).kinematics_estimated.position
            object_position = self.client.simGetObjectPose(
                object_name=self.target_drone
            ).position

            # Extract coordinates
            x, y, z = estimated_position.x_val, estimated_position.y_val, estimated_position.z_val
            ox, oy, oz = object_position.x_val, object_position.y_val, object_position.z_val

            # Check proximity to the current line segment
            if len(self.point_queue) >= 2:
                current_line_a = self.point_queue[0]
                current_line_b = self.point_queue[1]
                current_position = [x, y, z]

                if not GeoUtil.is_point_close_to_line(
                    current_line_a, current_line_b, current_position, self.deviation_percentage
                ):
                    # Log the first breach
                    if not self.breach_flag:
                        self.append_fail_to_log(
                            f"{self.target_drone}; First breach, deviation > {self.deviation_percentage} meters"
                        )
                        self.breach_flag = True

                # Move to the next segment if the waypoint is reached
                if GeoUtil.get_distance_btw_3d_points(current_line_b, current_position) < self.deviation_percentage:
                    self.point_queue.pop(0)

            # Append positions to tracking arrays
            self.est_position_array.append([x, y, z])
            self.obj_position_array.append([ox, oy, oz])

            sleep(self.dt)  # Pause for the update interval

    def calculate_actual_distance(self):
        """
        Calculates the actual distance traveled by the drone based on its estimated positions.

        Returns:
            float: The total actual traveled distance.
        """
        distance = 0.0
        for i in range(1, len(self.est_position_array)):
            distance += self.get_distance_btw_points(
                self.est_position_array[i - 1], self.est_position_array[i]
            )
        return distance

    def draw_trace_3d(self):
        """
        Generates a 3D visualization comparing the planned and actual flight paths.

        Graphs are saved in the target directory.
        """
        graph_dir = self.get_graph_dir()

        if not self.breach_flag:
            title = (
                f"{self.target_drone} Planned vs. Actual\n"
                f"Drone speed: {self.mission.speed} m/s\n"
                f"Wind: {self.wind_speed_text}"
            )
        else:
            title = (
                f"(FAILED) {self.target_drone} Planned vs. Actual\n"
                f"Drone speed: {self.mission.speed} m/s\n"
                f"Wind: {self.wind_speed_text}"
            )

        grapher = ThreeDimensionalGrapher()

        # Generate static and interactive 3D graphs
        grapher.draw_trace_vs_planned(
            planned_position_list=self.mission.points,
            actual_position_list=self.est_position_array,
            full_target_directory=graph_dir,
            drone_name=self.target_drone,
            title=title
        )

        grapher.draw_interactive_trace_vs_planned(
            planned_position_list=self.mission.points,
            actual_position_list=self.est_position_array,
            full_target_directory=graph_dir,
            drone_name=self.target_drone,
            title=title
        )
