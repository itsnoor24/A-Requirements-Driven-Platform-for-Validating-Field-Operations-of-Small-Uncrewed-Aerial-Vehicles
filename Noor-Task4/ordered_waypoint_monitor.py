from time import sleep
from PythonClient.multirotor.monitor.abstract.single_drone_mission_monitor import SingleDroneMissionMonitor


class OrderedWaypointMonitor(SingleDroneMissionMonitor):
    """
    Monitors a mission where the drone has to visit waypoints in a specific order.

    As the waypoints are reached it logs progress and verifies that all waypoints are visited
    in the correct order within a specified deviation threshold.
    """

    def __init__(self, mission, deviation_threshold=1):
        """
        Initialize the OrderedWaypointMonitor.

        Arguments:
            mission: The drone mission containing the waypoints.
            deviation_threshold (float): The allowable distance (in meters) to consider a waypoint reached.
        """
        super().__init__(mission)
        self.queue = None  # Queue to track remaining waypoints
        self.deviation_threshold = deviation_threshold
        self.target_drone = mission.target_drone

    def start(self):
        """
        Start monitoring the mission.

        Verifies mission compatibility, initializes the waypoint queue,
        and begins tracking progress through the waypoints.
        """
        if type(self.mission).__name__ not in self.polygon_mission_names:
            # Incompatible mission types are skipped
            return

        # Initialize the waypoint queue
        self.queue = self.mission.points.copy()

        # Log the start of monitoring
        self.append_info_to_log(f"{self.target_drone}; OrderedWaypointMonitor started")

        # Monitor the queue for waypoint completion
        self.update_queue()

        # Finalize and log the results
        self.stop()

    def update_queue(self):
        """
        Continuously checks the drone's position to determine if the next waypoint
        in the sequence has been reached. Removes waypoints from the queue as they are reached.
        """
        dt = 0.1  # Update interval in seconds

        while self.mission.state != self.mission.State.END:
            # Get the drone's current position
            position = self.client.getMultirotorState(
                vehicle_name=self.target_drone
            ).kinematics_estimated.position
            current_position = (position.x_val, position.y_val, position.z_val)

            # Check if the drone has reached the next waypoint
            if self.get_distance_btw_points(current_position, self.queue[0]) <= self.deviation_threshold:
                self.append_info_to_log(
                    f"{self.target_drone}; Reached {self.queue[0]} within {self.deviation_threshold} meters"
                )
                self.queue.pop(0)  # Remove the reached waypoint from the queue

            # Break the loop if all waypoints are reached
            if not self.queue:
                break

            sleep(dt)  # Wait for the next update

    def stop(self):
        """
        Finalizes monitoring and logs the mission results.

        Reports success if all waypoints were reached in the right order,
        or failure if any waypoints were unvisited.
        """
        if not self.queue:
            # All waypoints successfully reached in order
            self.append_pass_to_log(f"{self.target_drone}; All points reached in order")
        else:
            # Log failure with remaining waypoints
            self.append_fail_to_log(
                f"{self.target_drone}; Some points were not reached in the correct order: {self.queue}"
            )

        # Save the monitoring report
        self.save_report()
