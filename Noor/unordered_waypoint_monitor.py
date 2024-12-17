from time import sleep
from PythonClient.multirotor.monitor.abstract.single_drone_mission_monitor import SingleDroneMissionMonitor


class UnorderedWaypointMonitor(SingleDroneMissionMonitor):
    """
    Monitors a misssion to make sure all waypoints have been reached without caring about specific order.

    As waypoints are reached, it logs progress and provides a final report on whether all waypoints were visited
    within a specified deviation threshold.
    """

    def __init__(self, mission, deviation_threshold=1):
        """
        Initializes the UnorderedWaypointMonitor.

        Arguments:
            mission: The drone mission to monitor.
            deviation_threshold (float): Maximum allowable distance from a waypoint for it to be considered reached.
        """
        super().__init__(mission)
        self.deviation_threshold = deviation_threshold
        self.point_dict = {}  # Dictionary to track the status of waypoints (True if reached)

    def start(self):
        """
        Starts the monitoring process. Initializes waypoint tracking and begins monitoring
        the drone's position compared to the waypoints.
        """
        if type(self.mission).__name__ not in self.polygon_mission_names:
            # Skip  incompatible mission types
            return

        # Initialize tracking for all waypoints
        self.init_dict()

        # Log the start of monitoring
        self.append_info_to_log(f"{self.target_drone}; UnorderedWaypointMonitor started")

      
        # Monitor waypoint progress
        self.update_dict()

        # Finalize and log results
        self.stop()

    def init_dict(self):
        """
        Initializes the waypoint tracking dictionary.

        Each waypoint in the mission is added to `point_dict` and marked as not yet reached (False).
        """
        for waypoint in self.mission.points:
            self.point_dict[waypoint] = False

    def update_dict(self):
        """
        Continuously checks the drone's position and updates the waypoint tracking dictionary.

        Waypoints are marked as 'reached' when the drone's position is within the deviation threshold.
        """
        dt = 0.1  # Update interval in seconds

        while self.mission.state != self.mission.State.END:
            # Get the current position of the drone
            position = self.client.getMultirotorState(vehicle_name=self.target_drone).kinematics_estimated.position
            current_position = (position.x_val, position.y_val, position.z_val)

            # Check each waypoint for proximity
            for waypoint in self.mission.points:
                if self.get_distance_btw_points(current_position, waypoint) <= self.deviation_threshold and not self.point_dict[waypoint]:
                    # Mark waypoint as reached and log it
                    self.append_info_to_log(
                        f"{self.target_drone}; Reached {waypoint} within {self.deviation_threshold} meters"
                    )
                    self.point_dict[waypoint] = True

            sleep(dt)  # Wait for the next position update

    def stop(self):
        """
        Finalizes the monitoring process and logs the results.

        Reports 'success' if all waypoints were reached, or failure with a list of the unreached waypoints.
        """
        if all(self.point_dict.values()):
            # All waypoints successfully reached
            self.append_pass_to_log(f"{self.target_drone}; All points reached")
        else:
            # Log unreached waypoints
            unreached = [waypoint for waypoint, reached in self.point_dict.items() if not reached]
            self.append_fail_to_log(
                f"{self.target_drone}; Not all points reached, unreached points: {unreached}"
            )

        # Save the final report
        self.save_report()
