import math
from time import sleep
import numpy as np

from PythonClient.multirotor.util.geo.geo_util import GeoUtil
from PythonClient.multirotor.util.graph.three_dimensional_grapher import (
    ThreeDimensionalGrapher,
)
from PythonClient.multirotor.monitor.abstract.single_drone_mission_monitor import (
    SingleDroneMissionMonitor,
)


class CircularDeviationMonitor(SingleDroneMissionMonitor):

    def __init__(self, mission, deviation_percentage=10):
        super().__init__(mission)
        self.deviation_percentage = deviation_percentage
        self.reported_breach = False
        self.breach_flag = False
        self.total_distance_diff = None
        self.passed = False
        self.obj_position_array = None
        self.est_position_array = None
        self.actual_distance = None
        self.optimal_distance = None
        self.mission = mission
        self.target = mission.target_drone

    def start(self):
        if type(self.mission).__name__ not in self.circular_mission_names:
            return
        self.log_info(
            f"{self.target_drone}; speed: {self.mission.speed} m/s with wind {self.wind_speed_text}"
        )
        self.optimal_distance = self.calculate_planned_distance()
        self.log_info(
            f"{self.target_drone}; optimal distance: {round(self.optimal_distance, 2)} meters"
        )

        while self.mission.state == self.mission.State.IDLE:
            pass

        self.update_position()
        self.stop()

    def stop(self):
        self.log_info(f"{self.target_drone}; Mission task finished")
        self.actual_distance = self.calculate_actual_distance()

        if self.breach_flag:
            self.log_fail(
                f"{self.target_drone}; Deviation Violation (>{self.deviation_percentage}%)"
            )
        else:
            self.log_pass(
                f"{self.target_drone}; No deviation violation (<{self.deviation_percentage}%)"
            )

        if self.actual_distance is None or self.actual_distance == 0:
            self.log_fail(f"{self.target_drone}; Actual distance is 0")
        else:
            self.total_distance_diff = (
                self.actual_distance - self.optimal_distance
            ) / self.optimal_distance
            self.draw_trace_3d()

        self.save_report()

    def calculate_planned_distance(self):
        target_height = self.mission.altitude
        distance = self.get_distance_between_points([0, 0, 0], [0, 0, target_height])
        distance += math.pi * 2 * self.mission.radius * self.mission.iterations
        return distance

    def update_position(self):
        dt = 0.01
        self.est_position_array = []
        self.obj_position_array = []

        while self.mission.state != self.mission.State.END:
            estimated_position = self.client.getMultirotorState(
                self.target_drone
            ).kinematics_estimated.position
            object_position = self.client.simGetObjectPose(self.target_drone).position

            x, y, z = (
                estimated_position.x_val,
                estimated_position.y_val,
                estimated_position.z_val,
            )
            ox, oy, oz = (
                object_position.x_val,
                object_position.y_val,
                object_position.z_val,
            )

            if self.mission.climbed and not self.check_breach(x, y, z):
                if not self.reported_breach:
                    self.reported_breach = True
                    self.log_fail(
                        f"{self.target_drone}; First breach: deviated more than {self.deviation_percentage}m from the planned route"
                    )
                self.breach_flag = True

            self.est_position_array.append([x, y, z])
            self.obj_position_array.append([ox, oy, oz])
            sleep(dt)

    def check_breach(self, x, y, z):
        return GeoUtil.is_point_close_to_circle(
            [
                self.mission.center.x_val,
                self.mission.center.y_val,
                self.mission.altitude,
            ],
            self.mission.radius,
            [x, y, -z],
            self.deviation_percentage,
        )

    def calculate_actual_distance(self):
        distance = 0.0
        for i in range(1, len(self.est_position_array)):
            distance += self.get_distance_between_points(
                self.est_position_array[i - 1], self.est_position_array[i]
            )
        return distance

    @staticmethod
    def get_distance_between_points(point_1, point_2):
        return math.sqrt(
            (point_2[0] - point_1[0]) ** 2
            + (point_2[1] - point_1[1]) ** 2
            + (point_2[2] - point_1[2]) ** 2
        )

    def draw_trace_3d(self):
        graph_dir = self.get_graph_dir()
        est_actual = self.est_position_array
        radius = self.mission.radius
        height = self.mission.altitude

        if not self.breach_flag:
            title = f"{self.target_drone} Planned vs. Actual\nDrone speed: {self.mission.speed} m/s\nWind: {self.wind_speed_text}"
        else:
            title = f"(FAILED) {self.target_drone} Planned vs. Actual\nDrone speed: {self.mission.speed} m/s\nWind: {self.wind_speed_text}"

        center = [self.mission.center.x_val, self.mission.center.y_val, height]
        theta = np.linspace(0, 2 * np.pi, 100)
        x = center[0] + radius * np.cos(theta)
        y = center[1] + radius * np.sin(theta)
        z = np.ones(100) * height

        planned = [[x[i], y[i], -z[i]] for i in range(len(x))]

        ThreeDimensionalGrapher.draw_trace_vs_planned(
            planned_position_list=planned,
            actual_position_list=est_actual,
            full_target_directory=graph_dir,
            drone_name=self.target_drone,
            title=title,
        )

        ThreeDimensionalGrapher.draw_interactive_trace_vs_planned(
            planned_position_list=planned,
            actual_position_list=est_actual,
            full_target_directory=graph_dir,
            drone_name=self.target_drone,
            title=title,
        )
