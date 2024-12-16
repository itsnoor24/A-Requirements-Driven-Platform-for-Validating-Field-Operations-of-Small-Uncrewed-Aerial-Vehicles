import threading
from time import sleep

from PythonClient.multirotor.mission.fly_to_points import FlyToPoints
from PythonClient.multirotor.monitor.abstract.single_drone_mission_monitor import SingleDroneMissionMonitor


class TorqueBatteryMonitor(SingleDroneMissionMonitor):
    """
    Monitors the battery charge of a drone during a mission, making sure that it stays above a specified threshold.
    It also logs warnings and failures if the battery falls below the threshold.
    """

    def __init__(self, mission, min_battery_percentage=15):
        """
        Initializes the battery monitor.

        Arguments:
            mission: The drone mission being monitored.
            min_battery_percentage (float): Minimum allowable battery percentage before logging a failure.
        """
        super().__init__(mission)
        self.min_charge = 100  # Start with 100% as the initial minimum charge
        self.min_battery_percentage = min_battery_percentage
        self.passed = True
        self.mission = mission
        self.target = mission.target_drone

    def start(self):
        """
        Starts the battery monitoring process. Monitors the battery during the mission
        and logs any failures if the battery falls below the minimum threshold.
        """
        self.append_info_to_log(
            f"{self.target_drone}; Speed: {self.mission.speed} m/s with wind: {self.wind_speed_text}"
        )

        # Wait for the mission to start
        while self.mission.state == self.mission.State.IDLE:
            pass

        # Monitor battery while the mission is active
        while self.mission.state != self.mission.State.END:
            charge = self.get_current_battery_percentage()
            self.min_charge = min(charge, self.min_charge)

            # Debug log to track battery percentage
            print(f"DEBUG: Current battery charge: {charge}%")

            # Log a failure if the battery is below the threshold
            if charge < self.min_battery_percentage:
                self.passed = False
                self.append_fail_to_log(
                    f"{self.target_drone}; Battery below {self.min_battery_percentage}%"
                )

            sleep(1)  # Update battery status every second

        # Finalize and stop monitoring
        self.stop()

    def stop(self):
        """
        Stops the monitoring process and logs the results.
        If the battery stayed above the minimum threshold, it logs a success.
        Otherwise, it logs a failure with the minimum charge reached.
        """
        if self.passed:
            self.append_pass_to_log(
                f"{self.target_drone}; Battery remained above {self.min_battery_percentage}%, "
                f"minimum charge was {round(self.min_charge, 2)}%"
            )
        else:
            self.append_fail_to_log(
                f"{self.target_drone}; Battery dropped below {self.min_battery_percentage}%"
            )

    def get_current_battery_percentage(self):
        """
        Retrieves the current battery percentage based on rotor states and calculates
        the remaining battery capacity during the mission.

        Returns:
            float: Estimated battery percentage.
        """
        rotor_states = self.client.getRotorStates(vehicle_name=self.target_drone).rotors

        # Retrieve thrust and torque values for calculations
        thrust_values = [rotor_states[i]["thrust"] for i in range(len(rotor_states))]
        torque_values = [rotor_states[i]["torque_scaler"] for i in range(len(rotor_states))]

        # Calculate the remaining battery capacity
        battery_remaining = self.calculate_power(
            thrust_values=thrust_values,
            speed=self.mission.speed,
            torque_values=torque_values,
            angular_velocity=0,  # Assuming zero angular velocity for this example
            battery_capacity=1000,  # Assumed battery capacity in arbitrary units
            time_step=1  # Time step in seconds
        )

        return battery_remaining / 1000  # Normalize to percentage

    @staticmethod
    def calculate_power(thrust_values, speed, torque_values, angular_velocity, battery_capacity, time_step):
        """
        Calculates the remaining battery capacity based on power consumption.

        Arguments:
            thrust_values (list): List of thrust values from the rotors.
            speed (float): Current speed of the drone.
            torque_values (list): List of torque values from the rotors.
            angular_velocity (float): Angular velocity of the drone (assumed zero here).
            battery_capacity (float): Total battery capacity.
            time_step (float): Simulation time step in seconds.

        Returns:
            float: Remaining battery capacity after the time step.
        """
        # Calculate power required for thrust and speed
        total_thrust = sum(thrust_values)
        power_altitude_speed = total_thrust * speed

        # Calculate power required for torque (rotational dynamics)
        total_torque = sum(torque_values)
        power_torque = total_torque * angular_velocity

        # Total power required
        total_power = power_altitude_speed + power_torque

        # Deduct power consumption from battery capacity
        battery_remaining = battery_capacity - (total_power * time_step)

        return battery_remaining


if __name__ == "__main__":
    # Example usage: Monitor battery during a FlyToPoints mission
    mission = FlyToPoints(target_drone="Drone 1")
    monitor = TorqueBatteryMonitor(mission)

    # Run the mission and monitor in separate threads
    mission_thread = threading.Thread(target=mission.start)
    monitor_thread = threading.Thread(target=monitor.start)

    mission_thread.start()
    monitor_thread.start()
