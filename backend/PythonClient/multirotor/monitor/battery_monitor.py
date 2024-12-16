from drone_mission_base import SingleDroneMissionMonitor  # simplified import


class MyDroneBatteryMonitor(SingleDroneMissionMonitor):
    def __init__(self, mission, battery_threshold=15):
        super().__init__(mission)
        self.battery_threshold = battery_threshold
        self.battery_ok = True
        self.mission = mission
        self.drone = mission.target_drone

    def start_monitoring(self):
        # Log initial flight conditions
        self.log_info(
            f"Drone: {self.drone} - Flight speed: {self.mission.speed} m/s, Wind: {self.wind_speed_text}"
        )

        # Wait for mission to start
        while self.mission.state == self.mission.IDLE:
            continue

        # Main monitoring loop
        while not self._is_mission_complete():
            battery_level = self._get_battery_level()

            if battery_level < self.battery_threshold:
                self.battery_ok = False
                self.log_error(
                    f"Warning: Battery below {self.battery_threshold}% on drone {self.drone}"
                )

        self._finish_monitoring()

    def _is_mission_complete(self):
        return self.mission.state == self.mission.END

    def _get_battery_level(self):
        return self.client.getTripStats().state_of_charge

    def _finish_monitoring(self):
        if self.battery_ok:
            self.log_info(
                f"Mission complete - Battery stayed above {self.battery_threshold}% for drone {self.drone}"
            )
        else:
            self.log_error(
                f"Mission ended - Battery dropped below {self.battery_threshold}% for drone {self.drone}"
            )

        self.save_report()
