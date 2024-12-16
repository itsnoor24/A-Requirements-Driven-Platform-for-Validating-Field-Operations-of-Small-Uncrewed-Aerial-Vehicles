import threading
from time import sleep
from PythonClient.multirotor.mission.fly_straight import FlyStraight
from PythonClient.multirotor.monitor.abstract.single_drone_mission_monitor import (
    SingleDroneMissionMonitor,
)


class CollisionMonitor(SingleDroneMissionMonitor):

    def __init__(self, mission):
        super().__init__(mission)

    def start(self):
        self.log_info(f"{self.target_drone}; CollisionMonitor started")
        prev_collision = None
        time_interval = 3.0
        collision_detected = False

        while self.mission.state != self.mission.State.END:
            collision_info = self.client.simGetCollisionInfo(self.target_drone)

            if collision_info.has_collided:
                if (
                    prev_collision is None
                    or prev_collision != collision_info.object_name
                ):
                    collision_location = self.client.simGetObjectPose(
                        self.target_drone
                    ).position
                    self.log_fail(
                        f"{self.target_drone}; Collided with {collision_info.object_name} "
                        f"at position x = {collision_location.x_val} m, "
                        f"y = {collision_location.y_val} m, "
                        f"z = {collision_location.z_val} m"
                    )

                    collision_detected = True
                    self.capture_image(collision_info.object_name)
                    prev_collision = collision_info.object_name

            sleep(time_interval)

        if not collision_detected:
            self.log_pass(f"{self.target_drone}; No collision detected")
        else:
            self.log_fail(f"{self.target_drone}; Collision detected")

        self.save_report()


if __name__ == "__main__":
    drone_speed = 14
    destination = [10, 10, -0.5]
    mission = FlyStraight(destination=destination, speed=drone_speed)
    monitor = CollisionMonitor(mission)

    # Launching the mission and monitor in parallel threads
    mission_thread = threading.Thread(target=mission.start)
    monitor_thread = threading.Thread(target=monitor.start)

    mission_thread.start()
    monitor_thread.start()
