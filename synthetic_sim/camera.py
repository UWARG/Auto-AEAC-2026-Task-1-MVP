"""
Synthetic camera that renders world based on drone position/orientation.

Uses perspective projection to convert 3D world → 2D camera image.
"""

import numpy as np
import cv2
from typing import Tuple, Optional, List
from dataclasses import dataclass

from synthetic_sim.coordinates import LocalCoord, CoordinateTransform, GPSCoord
from synthetic_sim.world import World, CircleTarget, Box, Color


@dataclass
class CameraConfig:
    """Camera parameters."""

    fov_deg: float = 90.0  # Field of view (degrees)
    pitch_deg: float = -90.0  # Camera pitch (-90 = down, 0 = forward, 90 = up)
    roll_deg: float = 0.0  # Camera roll (usually 0)
    image_width: int = 640
    image_height: int = 480


@dataclass
class TargetMetadata:
    """Ground truth data for a visible target."""

    gps_position: GPSCoord
    local_position: LocalCoord
    pixel_position: Tuple[int, int]
    color: Color
    distance_meters: float
    is_on_ground: bool
    is_on_wall: bool
    is_on_roof: bool


class SyntheticCamera:
    """
    Renders synthetic camera view based on drone position and world state.

    Usage:
        camera = SyntheticCamera(world, coord_transform, camera_config)
        frame, metadata = camera.render(drone_local_pos, drone_yaw_deg)
    """

    def __init__(
        self,
        world: World,
        coord_transform: CoordinateTransform,
        config: CameraConfig = None,
    ):
        """
        Initialize synthetic camera.

        Args:
            world: The simulated world to render
            coord_transform: Coordinate transformation (must be initialized)
            config: Camera configuration
        """
        self.world = world
        self.coord_transform = coord_transform
        self.config = config or CameraConfig()

        # Calculate focal length from FOV
        self.focal_length = (self.config.image_width / 2) / np.tan(
            np.radians(self.config.fov_deg / 2)
        )

    def _world_to_camera_coords(
        self, world_point: LocalCoord, drone_pos: LocalCoord, drone_yaw_deg: float
    ) -> Optional[Tuple[float, float, float]]:
        """
        Transform world point to camera coordinate system.

        Camera coords: X=forward, Y=right, Z=down (from camera perspective)

        Args:
            world_point: Point in world coordinates
            drone_pos: Drone position in world coordinates
            drone_yaw_deg: Drone heading (0=North, 90=East, 180=South, 270=West)

        Returns:
            (x_cam, y_cam, z_cam) or None if behind camera
        """
        # 1. Translate to drone-relative coordinates
        # World frame: X=North, Y=East, Z=UP
        dx = world_point.x - drone_pos.x
        dy = world_point.y - drone_pos.y
        dz = world_point.z - drone_pos.z

        # 2. Rotate by drone yaw (heading)
        # Convert from world frame (X=North, Y=East, Z=Up) to body frame (X=forward, Y=right, Z=up still)
        yaw_rad = np.radians(drone_yaw_deg)
        x_body = dx * np.cos(yaw_rad) + dy * np.sin(yaw_rad)  # Forward direction
        y_body = -dx * np.sin(yaw_rad) + dy * np.cos(yaw_rad)  # Right direction
        z_body = dz  # Still up

        # 3. Rotate by camera pitch to convert to camera frame
        # Camera frame: X=forward, Y=right, Z=DOWN
        # Body frame still has Z=UP, so we need to convert up→down
        # Pitch: negative = looking down, 0 = looking forward, positive = looking up
        #
        # Pitch rotation around Y axis:
        # When pitch = -90° (looking straight down):
        #   Body forward (x) → Camera down (z)
        #   Body up (z) → Camera backward (-x)
        #
        pitch_rad = np.radians(self.config.pitch_deg)
        z_body_down = -z_body  # Convert Z from UP to DOWN convention

        x_cam = x_body * np.cos(pitch_rad) - z_body_down * np.sin(pitch_rad)
        y_cam = y_body
        z_cam = x_body * np.sin(pitch_rad) + z_body_down * np.cos(pitch_rad)

        # 4. Check if point is in front of camera
        if x_cam <= 0.1:  # Must be at least 10cm in front
            return None

        return (x_cam, y_cam, z_cam)

    def _camera_to_pixel(
        self, cam_coords: Tuple[float, float, float]
    ) -> Tuple[int, int]:
        """
        Project camera coordinates to pixel coordinates.

        Args:
            cam_coords: (x_cam, y_cam, z_cam) where x=forward, y=right, z=down

        Returns:
            (pixel_x, pixel_y) or None if outside frame
        """
        x_cam, y_cam, z_cam = cam_coords

        # Perspective projection
        # y_cam > 0 means right → pixel_x increases (right side of image)
        # z_cam > 0 means down → pixel_y increases (bottom of image)
        pixel_x = int((y_cam / x_cam) * self.focal_length + self.config.image_width / 2)
        pixel_y = int(
            (z_cam / x_cam) * self.focal_length + self.config.image_height / 2
        )

        return (pixel_x, pixel_y)

    def _project_point(
        self, world_point: LocalCoord, drone_pos: LocalCoord, drone_yaw_deg: float
    ) -> Optional[Tuple[int, int, float]]:
        """
        Project world point to pixel coordinates.

        Returns:
            (pixel_x, pixel_y, distance) or None if not visible
        """
        cam_coords = self._world_to_camera_coords(world_point, drone_pos, drone_yaw_deg)
        if cam_coords is None:
            return None

        pixel = self._camera_to_pixel(cam_coords)
        if pixel is None:
            return None

        # Calculate distance
        distance = np.sqrt(cam_coords[0] ** 2 + cam_coords[1] ** 2 + cam_coords[2] ** 2)

        return (*pixel, distance)

    def _calculate_apparent_radius(self, actual_radius: float, distance: float) -> int:
        """Calculate how large a circle appears in pixels based on distance."""
        # Angular size = 2 * arctan(radius / distance)
        # Pixel size = focal_length * tan(angular_size / 2)
        if distance < 0.01:
            return 0

        apparent_size_rad = 2 * np.arctan(actual_radius / distance)
        pixel_radius = int(self.focal_length * np.tan(apparent_size_rad / 2))

        return max(1, pixel_radius)  # At least 1 pixel

    def _render_target(
        self,
        frame: np.ndarray,
        target: CircleTarget,
        drone_pos: LocalCoord,
        drone_yaw_deg: float,
    ) -> Optional[TargetMetadata]:
        """
        Render a single target and return its metadata if visible.

        Args:
            frame: Image to draw on
            target: Target to render
            drone_pos: Drone position
            drone_yaw_deg: Drone heading

        Returns:
            TargetMetadata if target is visible, None otherwise
        """
        result = self._project_point(target.position, drone_pos, drone_yaw_deg)

        if result:
            pixel_x, pixel_y, distance = result
            radius_px = self._calculate_apparent_radius(target.radius, distance)

            # Draw circle
            cv2.circle(
                frame, (pixel_x, pixel_y), radius_px, target.color.value, -1  # Filled
            )

            # Draw white outline
            cv2.circle(frame, (pixel_x, pixel_y), radius_px, (255, 255, 255), 2)

            # Store metadata
            if self.coord_transform.is_initialized():
                gps_pos = self.coord_transform.local_to_gps(target.position)
            else:
                gps_pos = GPSCoord(0, 0, 0)

            return TargetMetadata(
                gps_position=gps_pos,
                local_position=target.position,
                pixel_position=(pixel_x, pixel_y),
                color=target.color,
                distance_meters=distance,
                is_on_ground=target.is_on_ground(),
                is_on_wall=target.is_on_wall(),
                is_on_roof=target.is_on_roof(),
            )

        return None

    def render(
        self, drone_pos: LocalCoord, drone_yaw_deg: float
    ) -> Tuple[np.ndarray, List[TargetMetadata]]:
        """
        Render camera view from drone's perspective.

        Args:
            drone_pos: Drone position in local coordinates
            drone_yaw_deg: Drone heading (0=North, 90=East)

        Returns:
            frame: Rendered image (numpy array, BGR format)
            metadata: List of visible targets with ground truth data
        """
        # Create blank frame (sky blue background)
        frame = np.full(
            (self.config.image_height, self.config.image_width, 3),
            (200, 180, 130),  # Sky blue (BGR)
            dtype=np.uint8,
        )

        metadata = []

        # Render ground plane with grid
        self._render_ground(frame, drone_pos, drone_yaw_deg)

        # Render ground targets first (they'll be occluded by buildings)
        ground_targets = [t for t in self.world.get_all_targets() if t.is_on_ground()]
        for target in ground_targets:
            target_metadata = self._render_target(
                frame, target, drone_pos, drone_yaw_deg
            )
            if target_metadata:
                metadata.append(target_metadata)

        # Render buildings with their associated targets (proper occlusion via depth sorting)
        for box in self.world.get_all_boxes():
            box_metadata = self._render_box(frame, box, drone_pos, drone_yaw_deg)
            metadata.extend(box_metadata)

        # Draw crosshair at center
        center_x = self.config.image_width // 2
        center_y = self.config.image_height // 2
        cv2.line(
            frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 2
        )
        cv2.line(
            frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 2
        )
        cv2.circle(frame, (center_x, center_y), 40, (0, 255, 0), 2)

        return frame, metadata

    def _render_box(
        self, frame: np.ndarray, box: Box, drone_pos: LocalCoord, drone_yaw_deg: float
    ) -> List[TargetMetadata]:
        """
        Draw box with filled faces and associated targets.

        Returns:
            List of metadata for visible targets on this box
        """
        metadata = []

        corners = box.get_corners()
        projected = []
        cam_coords_list = []

        # Project all corners and store camera coordinates
        for corner in corners:
            cam_coords = self._world_to_camera_coords(corner, drone_pos, drone_yaw_deg)
            cam_coords_list.append(cam_coords)

            if cam_coords:
                pixel = self._camera_to_pixel(cam_coords)
                if pixel:
                    projected.append(pixel)
                else:
                    projected.append(None)
            else:
                projected.append(None)

        # Get all targets that are on walls or roof (not ground)
        box_targets = [t for t in self.world.get_all_targets() if not t.is_on_ground()]

        # Define faces with depth calculation
        faces = [
            # Each face: (corner indices, base_color, face_name)
            ([0, 1, 2, 3], (140, 140, 140), "bottom"),  # Bottom
            ([4, 5, 6, 7], (200, 200, 200), "top"),  # Top
            ([0, 1, 5, 4], (160, 160, 180), "south"),  # South face
            ([2, 3, 7, 6], (160, 160, 180), "north"),  # North face
            ([1, 2, 6, 5], (170, 170, 190), "east"),  # East face
            ([3, 0, 4, 7], (170, 170, 190), "west"),  # West face
        ]

        # Calculate average depth for each face and sort
        face_depths = []
        for indices, color, name in faces:
            # Calculate average camera X coordinate (depth) for this face
            depths = [
                cam_coords_list[i][0] if cam_coords_list[i] else float("inf")
                for i in indices
            ]
            if all(d != float("inf") for d in depths):
                avg_depth = sum(depths) / len(depths)
                face_depths.append((avg_depth, indices, color, name))

        # Sort faces by depth (farthest first)
        face_depths.sort(reverse=True, key=lambda x: x[0])

        # Draw faces from back to front, rendering targets on each face
        for avg_depth, indices, color, name in face_depths:
            face_points = [projected[i] for i in indices]

            # Check if ANY corner is visible
            visible_points = [p for p in face_points if p is not None]

            if len(visible_points) > 0:
                # If all corners are visible, draw full polygon
                if all(p is not None for p in face_points):
                    pts = np.array(face_points, dtype=np.int32)
                    cv2.fillPoly(frame, [pts], color)
                    cv2.polylines(frame, [pts], True, (60, 60, 60), 3)

                # If partially visible, project with extended bounds
                # OpenCV will clip the polygon to the frame automatically
                else:
                    # For points that project off-screen, use camera coords to estimate position
                    extended_points = []
                    for i, (point, cam_coord) in enumerate(
                        zip(face_points, [cam_coords_list[idx] for idx in indices])
                    ):
                        if point is not None:
                            extended_points.append(point)
                        elif (
                            cam_coord and cam_coord[0] > 0
                        ):  # In front of camera but off-screen
                            # Manually project without bounds checking
                            x_cam, y_cam, z_cam = cam_coord
                            px = int(
                                (y_cam / x_cam) * self.focal_length
                                + self.config.image_width / 2
                            )
                            py = int(
                                (z_cam / x_cam) * self.focal_length
                                + self.config.image_height / 2
                            )
                            # Clamp to reasonable extended bounds
                            px = max(-1000, min(self.config.image_width + 1000, px))
                            py = max(-1000, min(self.config.image_height + 1000, py))
                            extended_points.append((px, py))

                    # Draw if we have at least 3 points
                    if len(extended_points) >= 3:
                        pts = np.array(extended_points, dtype=np.int32)
                        cv2.fillPoly(frame, [pts], color)
                        cv2.polylines(frame, [pts], True, (60, 60, 60), 2)

                # Render targets on this face
                # Match targets to faces based on surface normal
                for target in box_targets:
                    target_on_this_face = False

                    # Check if target's surface normal matches this face
                    # For simplicity, check if target is on this specific face type
                    if name == "top" and target.is_on_roof():
                        target_on_this_face = True
                    elif (
                        name in ["north", "south", "east", "west"]
                        and target.is_on_wall()
                    ):
                        # Check if target position matches this face's position
                        # This is approximate - targets should be within the face bounds
                        face_info = box.get_faces()[name]
                        face_center = face_info["center"]

                        # Check if target is approximately on this face
                        # (within 0.5m of face plane)
                        if name == "north":
                            target_on_this_face = (
                                abs(target.position.x - face_center.x) < 0.5
                            )
                        elif name == "south":
                            target_on_this_face = (
                                abs(target.position.x - face_center.x) < 0.5
                            )
                        elif name == "east":
                            target_on_this_face = (
                                abs(target.position.y - face_center.y) < 0.5
                            )
                        elif name == "west":
                            target_on_this_face = (
                                abs(target.position.y - face_center.y) < 0.5
                            )

                    if target_on_this_face:
                        target_metadata = self._render_target(
                            frame, target, drone_pos, drone_yaw_deg
                        )
                        if target_metadata:
                            metadata.append(target_metadata)

        return metadata

    def _render_ground(
        self, frame: np.ndarray, drone_pos: LocalCoord, drone_yaw_deg: float
    ):
        """Render ground plane with fixed grid pattern."""
        grid_spacing = 10  # 10 meters between grid lines
        grid_color = (100, 140, 100)  # Brighter green for visibility

        # Only draw lines close to drone for performance
        drone_x, drone_y = drone_pos.x, drone_pos.y
        view_distance = 60  # meters

        # Calculate grid line positions aligned to world coordinates (not drone position)
        # Round to nearest grid_spacing to keep lines fixed in world space
        x_min = int((drone_x - view_distance) / grid_spacing) * grid_spacing
        x_max = int((drone_x + view_distance) / grid_spacing) * grid_spacing
        y_min = int((drone_y - view_distance) / grid_spacing) * grid_spacing
        y_max = int((drone_y + view_distance) / grid_spacing) * grid_spacing

        # Draw grid lines parallel to X axis (lines running East-West)
        for x in range(x_min, x_max + grid_spacing, grid_spacing):
            line_points = []
            # Sample along the Y axis for this X line
            for y in range(y_min, y_max + 1, 2):  # Sample every 2m along line
                point = LocalCoord(x, y, self.world.ground.altitude)
                result = self._project_point(point, drone_pos, drone_yaw_deg)
                if result:
                    line_points.append((result[0], result[1]))

            # Draw polyline if we have at least 2 points
            if len(line_points) >= 2:
                pts = np.array(line_points, dtype=np.int32)
                cv2.polylines(frame, [pts], False, grid_color, 1, cv2.LINE_AA)

        # Draw grid lines parallel to Y axis (lines running North-South)
        for y in range(y_min, y_max + grid_spacing, grid_spacing):
            line_points = []
            # Sample along the X axis for this Y line
            for x in range(x_min, x_max + 1, 2):  # Sample every 2m along line
                point = LocalCoord(x, y, self.world.ground.altitude)
                result = self._project_point(point, drone_pos, drone_yaw_deg)
                if result:
                    line_points.append((result[0], result[1]))

            # Draw polyline if we have at least 2 points
            if len(line_points) >= 2:
                pts = np.array(line_points, dtype=np.int32)
                cv2.polylines(frame, [pts], False, grid_color, 1, cv2.LINE_AA)
