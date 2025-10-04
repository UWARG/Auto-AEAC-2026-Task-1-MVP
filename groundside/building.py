"""
Building geometry reconstruction and target description generation for groundside.

This module receives building corner data from the drone and uses it to generate
human-readable descriptions of target locations.
"""

import logging
import math
from util import Coordinate, ALTITUDE_TOLERANCE_M


class Building:
    """Reconstructs building geometry and generates target location descriptions."""

    def __init__(self):
        """Initialize building with empty corner data."""
        self.corners: list[Coordinate] = []

    def is_complete(self) -> bool:
        """Check if building has all 4 corners."""
        return len(self.corners) == 4

    @property
    def height(self) -> float:
        """Extract building height from corner altitudes."""
        if self.corners:
            return self.corners[0].alt
        return 0.0

    def generate_target_description(self, target: Coordinate, colour: str) -> str:
        """
        Generate human-readable description of target location.

        Examples:
        - "Target is on the north face of the building, 3.2m above ground and 1.6m from the western wall. The colour is blue."
        - "Target is on the ground, 5.2m away from the west face of the building and 0.2m from the western wall when facing it from the outside. The colour is green."
        """
        if not self.is_complete():
            return f"Target location: {target} (colour: {colour}) - Building not fully mapped"

        # Determine if target is on ground or on building
        if target.alt < ALTITUDE_TOLERANCE_M:
            return self._generate_ground_target_description(target, colour)
        else:
            return self._generate_wall_target_description(target, colour)

    def _generate_wall_target_description(self, target: Coordinate, colour: str) -> str:
        """Generate description for target on building wall or roof."""
        # Check if target is on roof (at building height)
        if abs(target.alt - self.height) < ALTITUDE_TOLERANCE_M:
            return self._generate_roof_target_description(target, colour)

        # Find which wall the target is on
        face_info = self.get_face_info(target)

        if face_info is None:
            return f"Target (colour: {colour}) at {target} - Unable to determine face"

        orientation = face_info["orientation"]
        height = face_info["height_above_ground"]
        dist_from_left = face_info["distance_from_left_edge"]

        return (
            f"Target is on the {orientation} face of the building, "
            f"{height:.1f}m above ground and {dist_from_left:.1f}m from the "
            f"{self._get_reference_direction(orientation, 'left')} wall. "
            f"The colour is {colour}"
        )

    def _generate_roof_target_description(self, target: Coordinate, colour: str) -> str:
        """Generate description for target on roof."""
        # Calculate position relative to building corners
        # Find distances from edges
        min_distances = []
        for i in range(4):
            wall_start = self.corners[i]
            wall_end = self.corners[(i + 1) % 4]

            _, distance = self._distance_to_line_segment(target, wall_start, wall_end)
            orientation = self._get_wall_orientation(wall_start, wall_end)
            min_distances.append((distance, orientation, i))

        if not min_distances:
            return f"Target on roof (colour: {colour})"

        # Sort by distance to find closest two edges
        min_distances.sort()
        closest_edge = min_distances[0]
        second_edge = min_distances[1]

        return (
            f"Target is on the roof, {closest_edge[0]:.1f}m from the {closest_edge[1]} edge "
            f"and {second_edge[0]:.1f}m from the {second_edge[1]} edge. "
            f"The colour is {colour}"
        )

    def _generate_ground_target_description(
        self, target: Coordinate, colour: str
    ) -> str:
        """Generate description for target on ground."""
        # Find closest building face
        min_distance = float("inf")
        closest_face_idx = -1
        closest_point = None

        for i in range(4):
            wall_start = self.corners[i]
            wall_end = self.corners[(i + 1) % 4]

            point_on_wall, distance = self._distance_to_line_segment(
                target, wall_start, wall_end
            )

            if distance < min_distance:
                min_distance = distance
                closest_face_idx = i
                closest_point = point_on_wall

        if closest_face_idx == -1 or closest_point is None:
            return f"Target on ground at {target} (colour: {colour})"

        wall_start = self.corners[closest_face_idx]
        wall_end = self.corners[(closest_face_idx + 1) % 4]

        orientation = self._get_wall_orientation(wall_start, wall_end)
        dist_from_left = math.sqrt(self._distance_squared(closest_point, wall_start))

        return (
            f"Target is on the ground, {min_distance:.1f}m away from the {orientation} face "
            f"of the building and {dist_from_left:.1f}m from the "
            f"{self._get_reference_direction(orientation, 'left')} wall "
            f"when facing it from the outside. The colour is {colour}"
        )

    def get_face_info(self, target: Coordinate) -> dict | None:
        """
        Determine which building face a target is on and calculate distances.

        Returns dict with:
        - face_index: Which wall (0-3)
        - orientation: Cardinal direction (north/south/east/west)
        - height_above_ground: Distance from ground level
        - distance_from_left_edge: Distance from left corner when facing wall from outside
        """
        if not self.is_complete():
            logging.warning("Building corners not fully mapped")
            return None

        # Find closest wall to target
        min_distance = float("inf")
        closest_face = -1
        closest_point_on_wall = None

        for i in range(4):
            wall_start = self.corners[i]
            wall_end = self.corners[(i + 1) % 4]

            point_on_wall, distance = self._distance_to_line_segment(
                target, wall_start, wall_end
            )

            if distance < min_distance:
                min_distance = distance
                closest_face = i
                closest_point_on_wall = point_on_wall

        if closest_face == -1 or closest_point_on_wall is None:
            return None

        # Calculate face orientation
        wall_start = self.corners[closest_face]
        wall_end = self.corners[(closest_face + 1) % 4]

        orientation = self._get_wall_orientation(wall_start, wall_end)

        # Calculate distance from left edge (when facing wall)
        dist_from_left = math.sqrt(
            self._distance_squared(closest_point_on_wall, wall_start)
        )

        return {
            "face_index": closest_face,
            "orientation": orientation,
            "height_above_ground": target.alt,
            "distance_from_left_edge": dist_from_left,
        }

    def _distance_to_line_segment(
        self, point: Coordinate, line_start: Coordinate, line_end: Coordinate
    ) -> tuple[Coordinate, float]:
        """
        Calculate perpendicular distance from point to line segment.

        Returns: (closest_point_on_segment, distance)
        """
        # Vector from line_start to line_end
        line_vec_x = line_end.lon - line_start.lon
        line_vec_y = line_end.lat - line_start.lat

        # Vector from line_start to point
        point_vec_x = point.lon - line_start.lon
        point_vec_y = point.lat - line_start.lat

        # Calculate projection parameter
        line_length_squared = line_vec_x**2 + line_vec_y**2

        if line_length_squared < 1e-10:
            # Degenerate line segment (start == end)
            return line_start, math.sqrt(self._distance_squared(point, line_start))

        # Project point onto line (parameter t)
        t = (point_vec_x * line_vec_x + point_vec_y * line_vec_y) / line_length_squared

        # Clamp t to [0, 1] to stay on segment
        t = max(0.0, min(1.0, t))

        # Find closest point on segment
        closest = Coordinate(
            lat=line_start.lat + t * line_vec_y,
            lon=line_start.lon + t * line_vec_x,
            alt=line_start.alt,
        )

        distance = math.sqrt(self._distance_squared(point, closest))
        return closest, distance

    def _distance_squared(self, p1: Coordinate, p2: Coordinate) -> float:
        """Calculate squared Euclidean distance between two coordinates."""
        return (p1.lat - p2.lat) ** 2 + (p1.lon - p2.lon) ** 2

    def _get_wall_orientation(
        self, wall_start: Coordinate, wall_end: Coordinate
    ) -> str:
        """Determine cardinal direction of wall face (north, south, east, west)."""
        # Calculate wall vector
        dx = wall_end.lon - wall_start.lon  # East-West
        dy = wall_end.lat - wall_start.lat  # North-South

        # Determine which axis dominates to find wall orientation
        if abs(dx) > abs(dy):
            # Wall runs East-West, face points North or South
            return "north" if dy >= 0 else "south"
        else:
            # Wall runs North-South, face points East or West
            return "east" if dx >= 0 else "west"

    def _get_reference_direction(self, face_orientation: str, relative: str) -> str:
        """Get reference direction (e.g., 'left' when facing north is 'western')."""
        # Map face orientation + relative position to cardinal direction
        direction_map = {
            ("north", "left"): "western",
            ("north", "right"): "eastern",
            ("south", "left"): "eastern",
            ("south", "right"): "western",
            ("east", "left"): "northern",
            ("east", "right"): "southern",
            ("west", "left"): "southern",
            ("west", "right"): "northern",
        }
        return direction_map.get((face_orientation, relative), "unknown")

    def __str__(self):
        return f"Building(corners={self.corners}, height={self.height})"
