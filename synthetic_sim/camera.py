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
class RenderableFace:
    """A face of a building with its associated targets."""

    corners: List[LocalCoord]  # 4 corners of the face
    color: Tuple[int, int, int]  # BGR color
    normal: Tuple[float, float, float]  # Surface normal
    center: LocalCoord  # Center point of face
    targets: List[CircleTarget]  # Targets on this face
    name: str  # Face name for debugging


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

        # Extract all faces from all buildings
        all_faces = self._extract_all_faces()

        # Filter faces based on drone position (back-face culling)
        visible_faces = self._filter_visible_faces(all_faces, drone_pos)

        # Sort faces by depth (farthest first)
        sorted_faces = self._sort_faces_by_depth(visible_faces, drone_pos, drone_yaw_deg)

        # Render each face with its targets
        for face in sorted_faces:
            face_metadata = self._render_face(frame, face, drone_pos, drone_yaw_deg)
            metadata.extend(face_metadata)

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

    def _extract_all_faces(self) -> List[RenderableFace]:
        """
        Extract all faces from all buildings with their associated targets.

        Returns:
            List of RenderableFace objects
        """
        faces = []
        all_targets = self.world.get_all_targets()

        for box in self.world.get_all_boxes():
            corners = box.get_corners()
            face_info = box.get_faces()

            # Define face corner indices and properties
            # Corners: 0=SW-bottom, 1=NW-bottom, 2=NE-bottom, 3=SE-bottom
            #          4=SW-top,    5=NW-top,    6=NE-top,    7=SE-top
            # Where: N=+X (cx+hw), S=-X (cx-hw), E=+Y (cy+hd), W=-Y (cy-hd)
            face_defs = [
                ("bottom", [0, 1, 2, 3], (140, 140, 140)),
                ("top", [4, 5, 6, 7], (200, 200, 200)),
                ("south", [0, 3, 7, 4], (160, 160, 180)),
                ("north", [1, 2, 6, 5], (160, 160, 180)),
                ("east", [2, 3, 7, 6], (170, 170, 190)),
                ("west", [0, 1, 5, 4], (170, 170, 190)),
            ]

            for face_name, corner_indices, color in face_defs:
                face_corners = [corners[i] for i in corner_indices]
                info = face_info[face_name]

                # Find targets on this face
                face_targets = self._find_targets_for_face(
                    all_targets, face_name, info["center"], info["normal"]
                )

                faces.append(
                    RenderableFace(
                        corners=face_corners,
                        color=color,
                        normal=info["normal"],
                        center=info["center"],
                        targets=face_targets,
                        name=face_name,
                    )
                )

        return faces

    def _find_targets_for_face(
        self,
        all_targets: List[CircleTarget],
        face_name: str,
        face_center: LocalCoord,
        face_normal: Tuple[float, float, float],
    ) -> List[CircleTarget]:
        """
        Find all targets that belong to a specific face.

        Args:
            all_targets: All targets in the world
            face_name: Name of the face
            face_center: Center position of the face
            face_normal: Normal vector of the face

        Returns:
            List of targets on this face
        """
        face_targets = []

        for target in all_targets:
            # Skip ground targets
            if target.is_on_ground():
                continue

            # Check if target's surface normal matches this face's normal
            if target.surface_normal != face_normal:
                continue

            # Check if target position is on this face (within 0.5m tolerance)
            on_this_face = False

            if face_name in ["north", "south"]:
                # X-aligned face: check X coordinate
                on_this_face = abs(target.position.x - face_center.x) < 0.5
            elif face_name in ["east", "west"]:
                # Y-aligned face: check Y coordinate
                on_this_face = abs(target.position.y - face_center.y) < 0.5
            elif face_name == "top":
                # Top face: check Z coordinate
                on_this_face = abs(target.position.z - face_center.z) < 0.5

            if on_this_face:
                face_targets.append(target)

        return face_targets

    def _filter_visible_faces(
        self, faces: List[RenderableFace], drone_pos: LocalCoord
    ) -> List[RenderableFace]:
        """
        Filter faces based on drone position (back-face culling).

        Args:
            faces: All faces
            drone_pos: Drone position

        Returns:
            List of visible faces (facing toward the drone)
        """
        visible = []

        for face in faces:
            # Vector from face center to drone
            to_drone = np.array(
                [
                    drone_pos.x - face.center.x,
                    drone_pos.y - face.center.y,
                    drone_pos.z - face.center.z,
                ]
            )

            # Dot product with face normal
            # Positive means face is pointing toward drone
            dot = np.dot(to_drone, face.normal)

            if dot > 0:
                visible.append(face)

        return visible

    def _sort_faces_by_depth(
        self,
        faces: List[RenderableFace],
        drone_pos: LocalCoord,
        drone_yaw_deg: float,
    ) -> List[RenderableFace]:
        """
        Sort faces by average depth (farthest first).

        Args:
            faces: Faces to sort
            drone_pos: Drone position
            drone_yaw_deg: Drone heading

        Returns:
            Sorted list of faces (farthest to nearest)
        """
        face_depths = []

        for face in faces:
            # Calculate average depth of face corners
            depths = []
            for corner in face.corners:
                cam_coords = self._world_to_camera_coords(
                    corner, drone_pos, drone_yaw_deg
                )
                if cam_coords:
                    depths.append(cam_coords[0])  # x_cam is depth

            if depths:
                avg_depth = sum(depths) / len(depths)
                face_depths.append((avg_depth, face))

        # Sort by depth (farthest first)
        face_depths.sort(reverse=True, key=lambda x: x[0])

        return [face for _, face in face_depths]

    def _render_face(
        self,
        frame: np.ndarray,
        face: RenderableFace,
        drone_pos: LocalCoord,
        drone_yaw_deg: float,
    ) -> List[TargetMetadata]:
        """
        Render a single face with its targets.

        Args:
            frame: Image to draw on
            face: Face to render
            drone_pos: Drone position
            drone_yaw_deg: Drone heading

        Returns:
            List of metadata for visible targets on this face
        """
        metadata = []

        # Project face corners
        projected = []
        cam_coords_list = []

        for corner in face.corners:
            cam_coords = self._world_to_camera_coords(corner, drone_pos, drone_yaw_deg)
            cam_coords_list.append(cam_coords)

            if cam_coords:
                pixel = self._camera_to_pixel(cam_coords)
                projected.append(pixel)
            else:
                projected.append(None)

        # Check if any corner is visible
        visible_points = [p for p in projected if p is not None]

        if len(visible_points) > 0:
            # If all corners are visible, draw full polygon
            if all(p is not None for p in projected):
                pts = np.array(projected, dtype=np.int32)
                cv2.fillPoly(frame, [pts], face.color)
                cv2.polylines(frame, [pts], True, (60, 60, 60), 3)

            # If partially visible, project with extended bounds
            else:
                extended_points = []
                for point, cam_coord in zip(projected, cam_coords_list):
                    if point is not None:
                        extended_points.append(point)
                    elif cam_coord and cam_coord[0] > 0:
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
                    cv2.fillPoly(frame, [pts], face.color)
                    cv2.polylines(frame, [pts], True, (60, 60, 60), 2)

            # Render targets on this face
            for target in face.targets:
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
