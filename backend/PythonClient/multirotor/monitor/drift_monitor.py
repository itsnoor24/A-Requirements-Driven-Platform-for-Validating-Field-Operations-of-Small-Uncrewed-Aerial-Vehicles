import threading
from time import sleep
from PythonClient import airsim
from PythonClient.multirotor.util.graph.three_dimensional_grapher import (
    ThreeDimensionalGrapher,
)
from PythonClient.multirotor.mission.fly_straight import FlyStraight
from PythonClient.multirotor.monitor.abstract.single_drone_mission_monitor import (
    SingleDroneMissionMonitor,
)


class DriftMonitor(SingleDroneMissionMonitor):

    def __init__(self, mission, threshold=1, dt=0.01):
        super().__init__(mission)
        self.closest = None
        self.reached = False
        self.est_position_array = []
        self.mission = mission
        self.target_drone = mission.target_drone
        self.threshold = threshold
        self.dt = dt

    def start(self):
        if type(self.mission).__name__ not in self.point_mission_names:
            return

        self.track_drift()
        self.plot_3d_trace()
        self.save_report()

    def track_drift(self):
        closest_distance = float("inf")
        while self.mission.state != self.mission.State.END:
            current_position = self.client.getMultirotorState(
                vehicle_name=self.target_drone
            ).kinematics_estimated.position
            x, y, z = (
                current_position.x_val,
                current_position.y_val,
                current_position.z_val,
            )
            self.est_position_array.append([x, y, z])

            distance_to_target = self.calculate_distance([x, y, z], self.mission.point)
            closest_distance = min(closest_distance, distance_to_target)
            self.closest = closest_distance

            if distance_to_target < self.threshold or self.reached:
                if not self.reached:
                    self.reached = True

            sleep(self.dt)

        self.log_result(closest_distance)

    def calculate_distance(self, point1, point2):
        """
        Calculates the Euclidean distance between two points.
        """
        return sum((p1 - p2) ** 2 for p1, p2 in zip(point1, point2)) ** 0.5

    def log_result(self, closest_distance):
        """
        Logs whether the drone reached the target location within the threshold distance.
        """
        if not self.reached:
            self.append_fail_to_log(
                f"{self.target_drone}; Did NOT reach target location {self.mission.point} "
                f"within {self.threshold} meters. Closest distance: {round(closest_distance, 2)} meters"
            )
        else:
            self.append_pass_to_log(
                f"{self.target_drone}; Reached target location {self.mission.point} within "
                f"{self.threshold} meters. Closest distance: {round(closest_distance, 2)} meters"
            )

    def plot_3d_trace(self):
        """
        Plots the 3D trace of the drone's movement and the target location.
        """
        actual_positions = self.est_position_array
        target_point = self.mission.point
        title = (
            f"Drift path\nDrone speed: {self.mission.speed} m/s\nWind: {self.wind_speed_text}\n"
            f"Closest distance: {round(self.closest, 2)} meters"
            if self.reached
            else f"(FAILED) Drift path\nDrone speed: {self.mission.speed} m/s\nWind: {self.wind_speed_text}\n"
            f"Closest distance: {round(self.closest, 2)} meters"
        )

        graph_dir = self.get_graph_dir()
        grapher = ThreeDimensionalGrapher()
        grapher.draw_trace_vs_point(
            destination_point=target_point,
            actual_position_list=actual_positions,
            full_target_directory=graph_dir,
            drone_name=self.target_drone,
            title=title,
        )
        grapher.draw_interactive_trace_vs_point(
            actual_position=actual_positions,
            destination=target_point,
            full_target_directory=graph_dir,
            drone_name=self.target_drone,
            title=title,
        )


if __name__ == "__main__":

    drone_speed = 14
    target_point = [10, 10, -10]
    wind = airsim.Vector3r(20, 0, 0)
    mission = FlyStraight(destination=target_point, speed=drone_speed)
    mission.client.simSetWind(wind)
    monitor = DriftMonitor(mission)
    mission_thread = threading.Thread(target=mission.start)
    monitor_thread = threading.Thread(target=monitor.start)

    mission_thread.start()
    monitor_thread.start()
