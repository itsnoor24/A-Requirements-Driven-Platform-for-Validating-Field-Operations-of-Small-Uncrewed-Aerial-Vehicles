from time import sleep
import numpy as np
from numpy import concatenate
from scipy.spatial import ConvexHull

from PythonClient.multirotor.monitor.abstract.single_drone_mission_monitor import SingleDroneMissionMonitor
from PythonClient.multirotor.util.geo.geo_util import GeoUtil


class NoFlyZoneMonitor(SingleDroneMissionMonitor):
    """
    A monitor to make sure that the drone avoids predefined no-fly zones during its mission.

    The researchers use convex hulls to represent no-fly zones and check if the drone's position
    enters these zones during the mission. The violations are logged, and then the mission result is reported.
    """

    def __init__(self, mission, zone_polyhedra=None):
        """
        Initializes the NoFlyZoneMonitor.

        Arguments:
            mission: The mission to be monitored.
            zone_polyhedra (list, optional): A list of no-fly zones defined by geo-coordinates.
                                    It defaults to predefined no-fly zones if None.
        """
        super().__init__(mission)

        if zone_polyhedra is None:
            # Use default zones if none are provided
            self.zone_polyhedra_cartesian = self.default_cartesian_zones()
        else:
            if self.input_zone_polyhedra_is_valid(zone_polyhedra):
                print("DEBUG: Input zone polyhedra is valid")
                self.zone_polyhedra_cartesian = self.convert_to_cartesian(zone_polyhedra)
                self.zone_polyhedra_geo = zone_polyhedra
            else:
                print("Invalid zone polyhedra input, using default zones")
                self.zone_polyhedra_cartesian = self.default_cartesian_zones()
                self.append_fail_to_log(f"{self.target_drone}; Invalid zone polyhedra input, using default zones")

        # Generate convex hulls for the no-fly zones
        self.zone_polyhedra_hulls = self.make_poly_hulls()

    @staticmethod
    def default_cartesian_zones():
        """
        Returns a default set of no-fly zones in cartesian coordinates.

        Returns:
            list: A predefined list of 3D points representing no-fly zones.
        """
        return [
            # Zone 1
            [
                [1, -10, 0], [1, 10, 0], [5, -10, 0], [5, 10, 0],
                [1, -10, 999], [1, 10, 999], [5, -10, 999], [5, 10, 999]
            ],
            # Zone 2
            [
                [-1, -10, 0], [-1, 10, 0], [-5, -10, 0], [-5, 10, 0],
                [-1, -10, 999], [-1, 10, 999], [-5, -10, 999], [-5, 10, 999]
            ]
        ]

    def convert_to_cartesian(self, zone_polyhedra):
        """
        Converts a list of geo-coordinates to cartesian coordinates based on the simulation origin.

        Args:
            zone_polyhedra (list): List of geo-coordinates defining no-fly zones.

        Returns:
            list: A list of cartesian coordinates representing the zones.
        """
        cartesian_zones = []
        for zone in zone_polyhedra:
            cartesian_zone = [
                GeoUtil.geo_to_cartesian_coordinates(point[0], point[1], point[2], self.cesium_origin)
                for point in zone
            ]
            cartesian_zones.append(cartesian_zone)

        print("DEBUG: Converted cartesian zones = ", cartesian_zones)
        return cartesian_zones

    def start(self):
        """
        Starts monitoring the drone for no-fly zone violations during the mission.
        Logs whether the drone enters any no-fly zones.
        """
        self.append_info_to_log(f"{self.target_drone}; NoFlyZoneMonitor started")
        violation_flag = False

        while self.mission.state != self.mission.State.END:
            # Check the drone's current geographic position
            geo_point = self.get_current_geo_point()

            if self.is_in_zone(geo_point):
                violation_flag = True

            sleep(1)  # Update check every second

        # Log results based on the violation flag
        if violation_flag:
            self.append_fail_to_log(f"{self.target_drone}; NoFlyZoneMonitor ended with a violation")
        else:
            self.append_pass_to_log(f"{self.target_drone}; Mission completed, no violations detected")

        self.save_report()

    def get_current_abs_point(self):
        """
        Retrieves the drone's current position in coordinates.

        Returns:
            list: [x, y, z] coordinates of the drone's position.
        """
        position = self.client.simGetObjectPose(object_name=self.target_drone).position
        return [position.x_val, position.y_val, position.z_val]

    def get_current_geo_point(self):
        """
        Retrieves the drone's current position.

        Returns:
            list: [latitude, longitude, altitude] of the drone's position.
        """
        gps_data = self.client.getGpsData(vehicle_name=self.target_drone)
        return [gps_data.gnss.geo_point.latitude, gps_data.gnss.geo_point.longitude, gps_data.gnss.geo_point.altitude]

    def is_in_zone(self, point):
        """
        Checks if the given point is inside any of the defined no-fly zones.

        Arguments:
            point (list): [latitude, longitude, altitude] of the point to check.

        Returns:
            bool: True if the point is inside a no-fly zone, False if otherwise.
        """
        for hull in self.zone_polyhedra_hulls:
            # Add the point to the hull and check if it alters the convex hull
            new_hull = ConvexHull(concatenate((hull.points, [point])))
            if np.array_equal(new_hull.vertices, hull.vertices):
                self.append_fail_to_log(
                    f"{self.target_drone}; Entered no-fly zone at position "
                    f"[{round(point[0], 6)}, {round(point[1], 6)}, {round(point[2], 6)}]"
                )
                return True
        return False

    def make_poly_hulls(self):
        """
        Creates convex hulls for all defined no-fly zones.

        Returns:
            list: A list of ConvexHull objects representing no-fly zones.
        """
        try:
            hulls = []
            for zone in self.zone_polyhedra_cartesian:
                hulls.append(ConvexHull(points=np.array(zone)))

            self.append_info_to_log(
                f"{self.target_drone}; Created no-fly polyhedra zones: {self.zone_polyhedra_cartesian}"
            )
            return hulls
        except Exception as e:
            self.append_fail_to_log(
                f"{self.target_drone}; Failed to create no-fly zones: {type(e).__name__}. Using empty zones."
            )
            print("ERROR: Failed to create polyhedra hulls: ", type(e).__name__)
            return []

    @staticmethod
    def input_zone_polyhedra_is_valid(zone_polyhedra):
        """
        Validates the structure of the input zone polyhedra (the no fly zone).

        Arguments:
            zone_polyhedra (list): The input polyhedra to validate.

        Returns:
            bool: True if the input is valid, False if otherwise.
        """
        if not isinstance(zone_polyhedra, list):
            return False

        for zone in zone_polyhedra:
            if not isinstance(zone, list) or len(zone) < 4:
                return False
            for point in zone:
                if not isinstance(point, list) or len(point) != 3:
                    return False

        return True
