"""
Building geometry and target calculation utilities.

This module provides a Building class for managing building corner coordinates,
performing bounds checking, and calculating target points on building walls
based on drone position and heading.
"""

import logging
import math

from util import Coordinate, Vector3d


class Building:
    """Manages building geometry and calculates target points on walls."""

    def __init__(self):
        """Initialize building with empty corner data."""
        self.corners: list[Coordinate | None] = [None] * 4
        self.height: float = 0
        self.corner_record_cursor: int = 0

    def record_corner(self, corner: Coordinate):
        """Record a building corner coordinate in sequence."""
        self.corners[self.corner_record_cursor] = corner
        self.corner_record_cursor += 1
        if self.corner_record_cursor >= len(self.corners):
            logging.warning("Corner record cursor reached limit, wrapping around")
            self.corner_record_cursor = 0

    def record_height(self, height: float):
        """Record building height in meters."""
        self.height = height
        self.corners = [
            Coordinate(c.lat, c.lon, height) if c else None for c in self.corners
        ]

    def in_bounds(self, point: Coordinate) -> bool:
        """Check if a point is within the building's rectangular bounds."""
        if not all(self.corners):
            logging.warning("Not all corners are recorded, skipping bounds check")
            return False

        # Type guard: all corners are guaranteed to be non-None after the check above
        xs = [c.lat for c in self.corners if c is not None]
        ys = [c.lon for c in self.corners if c is not None]

        max_x, min_x = max(xs), min(xs)
        max_y, min_y = max(ys), min(ys)
        return min_x <= point.lat <= max_x and min_y <= point.lon <= max_y

    def find_target_on_wall(self, drone_pos: Coordinate, heading: float) -> Coordinate:
        """Find target coordinate on building wall directly in front of drone."""
        # Check if we have all corners mapped
        if not all(self.corners):
            logging.warning("Building corners not fully mapped, cannot find target")
            return Coordinate(lat=0, lon=0)

        # Convert heading to radians (0 degrees = North, 90 degrees = East)
        heading_rad = math.radians(heading)

        # Create ray direction vector using Vector3d
        # x = East component, y = North component, z = 0 (2D calculation)
        ray_direction = Vector3d(
            x=math.sin(heading_rad),  # East component
            y=math.cos(heading_rad),  # North component
            z=0.0,
        )

        # Find intersection with building walls
        # Building is rectangular, so we need to check intersection with 4 walls

        # Find the closest intersection point
        closest_intersection: Coordinate | None = None
        min_distance = float("inf")

        for i in range(4):
            wall_start = self.corners[i]
            wall_end = self.corners[(i + 1) % 4]

            # Type guard: corners are guaranteed to be non-None after the check above
            if wall_start is None or wall_end is None:
                continue

            intersection = self._ray_line_intersection(
                drone_pos, ray_direction, wall_start, wall_end
            )

            if intersection:
                # Calculate distance from drone to intersection
                distance = self._distance_squared(drone_pos, intersection)
                if distance < min_distance:
                    min_distance = distance
                    closest_intersection = intersection

        if closest_intersection:
            return closest_intersection
        else:
            logging.warning("No wall intersection found")
            return Coordinate(lat=0, lon=0)

    def _ray_line_intersection(
        self,
        ray_start: Coordinate,
        ray_direction: Vector3d,
        line_start: Coordinate,
        line_end: Coordinate,
    ) -> Coordinate | None:
        """
        Find intersection between a ray and a line segment using parametric equations.

        COORDINATE SYSTEM:
        - x-axis: East/West direction (affects longitude)
        - y-axis: North/South direction (affects latitude)
        - ray_direction.x = East component (sin(heading))
        - ray_direction.y = North component (cos(heading))

        MATHEMATICAL APPROACH:
        We solve for the intersection point using parametric equations:

        Ray equation:         P(t_ray) = ray_start + t_ray * ray_direction
        Line segment equation: L(t_seg) = line_start + t_seg * line_vector

        At intersection: P(t_ray) = L(t_seg)

        This gives us a system of 2 linear equations with 2 unknowns:
        ray_start.lon + t_ray * ray_direction.x = line_start.lon + t_seg * line_vector_x
        ray_start.lat + t_ray * ray_direction.y = line_start.lat + t_seg * line_vector_y

        Rearranging:
        t_ray * ray_direction.x - t_seg * line_vector_x = line_start.lon - ray_start.lon
        t_ray * ray_direction.y - t_seg * line_vector_y = line_start.lat - ray_start.lat

        Solving using Cramer's rule gives us t_ray and t_seg.

        VALID INTERSECTION CONDITIONS:
        - t_ray >= 0: Intersection is in front of the ray (forward direction)
        - 0 <= t_seg <= 1: Intersection is within the line segment bounds
        """
        # Calculate the line segment direction vector (from line_start to line_end)
        line_dx_x = line_end.lon - line_start.lon  # East/West (x-direction)
        line_dx_y = line_end.lat - line_start.lat  # North/South (y-direction)

        # Calculate the determinant (cross product of ray_direction and line_dx)
        # If zero, the ray and line are parallel (no unique intersection)
        determinant = ray_direction.x * line_dx_y - ray_direction.y * line_dx_x

        if abs(determinant) < 1e-10:
            return None

        # Calculate offset from line_start to ray_start
        dx_lon = ray_start.lon - line_start.lon  # East/West offset (x-direction)
        dx_lat = ray_start.lat - line_start.lat  # North/South offset (y-direction)

        # Solve for t_ray and t_seg
        t_ray = (line_dx_x * dx_lat - line_dx_y * dx_lon) / determinant
        t_seg = (ray_direction.x * dx_lat - ray_direction.y * dx_lon) / determinant

        # Check if intersection is valid:
        # - t_seg in [0,1]: intersection is on the line segment (not before start or after end)
        # - t_ray >= 0: intersection is in front of the ray (not behind the drone)
        if 0 <= t_seg <= 1 and t_ray >= 0:
            int_lat = ray_start.lat + t_ray * ray_direction.y  # North component
            int_lon = ray_start.lon + t_ray * ray_direction.x  # East component
            return Coordinate(lat=int_lat, lon=int_lon)

        return None

    def _distance_squared(self, p1: Coordinate, p2: Coordinate) -> float:
        """Calculate squared Euclidean distance between two coordinates."""
        return (p1.lat - p2.lat) ** 2 + (p1.lon - p2.lon) ** 2

    def __str__(self):
        return f"Building(corners={self.corners}, height={self.height})"
