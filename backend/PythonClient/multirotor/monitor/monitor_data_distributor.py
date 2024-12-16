import threading
import time
from collections import deque


class MonitorDataDistributor:
    def __init__(self, monitors, update_interval=1.0):
        """
        Initializes the MonitorDataDistributor with a list of monitors and the update interval.

        :param monitors: List of monitor objects that will receive the data.
        :param update_interval: Time in seconds between updates to each monitor.
        """
        self.monitors = monitors  # List of monitors to distribute data to
        self.update_interval = update_interval  # Time interval between updates
        self.data_queue = deque()  # Queue to hold the data before distributing it
        self.running = False  # Flag to control the update loop
        self.lock = (
            threading.Lock()
        )  # Lock to ensure thread safety while accessing shared resources

    def start(self):
        """
        Starts the data distribution process in a separate thread.
        """
        self.running = True
        self.distribution_thread = threading.Thread(target=self._distribute_data)
        self.distribution_thread.start()
        print("MonitorDataDistributor started")

    def stop(self):
        """
        Stops the data distribution process.
        """
        self.running = False
        if self.distribution_thread.is_alive():
            self.distribution_thread.join()
        print("MonitorDataDistributor stopped")

    def add_data(self, data):
        """
        Adds data to the distribution queue.

        :param data: The data to be distributed to monitors.
        """
        with self.lock:
            self.data_queue.append(data)

    def _distribute_data(self):
        """
        Continuously distributes data to each monitor at the specified update interval.
        """
        while self.running:
            if self.data_queue:
                with self.lock:
                    data = self.data_queue.popleft()

                for monitor in self.monitors:
                    monitor.receive_data(data)

            time.sleep(self.update_interval)


class Monitor:
    def __init__(self, name):
        """
        Initializes a monitor with a name.

        :param name: The name of the monitor.
        """
        self.name = name

    def receive_data(self, data):
        """
        Receives data from the distributor.

        :param data: The data being sent to the monitor.
        """
        print(f"{self.name} received data: {data}")


if __name__ == "__main__":
    # Example usage

    # Create some monitor instances
    monitor_1 = Monitor(name="Monitor 1")
    monitor_2 = Monitor(name="Monitor 2")
    monitor_3 = Monitor(name="Monitor 3")

    # Create a MonitorDataDistributor with a list of monitors
    distributor = MonitorDataDistributor(
        [monitor_1, monitor_2, monitor_3], update_interval=2.0
    )

    # Start the distributor
    distributor.start()

    # Add some data to the distributor
    distributor.add_data("Data Point 1")
    distributor.add_data("Data Point 2")
    distributor.add_data("Data Point 3")

    # Let it run for a while
    time.sleep(10)

    # Stop the distributor
    distributor.stop()
