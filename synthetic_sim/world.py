"""
Simulated world definition with primitives (ground, boxes, shapes on surfaces).

All objects are defined in local coordinates (meters) and can be locked
to GPS coordinates via CoordinateTransform.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Literal
from enum import Enum
from synthetic_sim.coordinates import LocalCoord


class Color(Enum):
    """Standard colors for targets."""
    RED = (0, 0, 255)      # BGR format for OpenCV
    BLUE = (255, 0, 0)
    GREEN = (0, 255, 0)
    YELLOW = (0, 255, 255)
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    GRAY = (128, 128, 128)


@dataclass
class Ground:
    """Ground plane at z=0."""
    color: Color = Color.GRAY
    altitude: float = 0.0  # meters above sea level


@dataclass
class Box:
    """Rectangular prism (building)."""
    center: LocalCoord      # Center of bottom face
    width: float            # X dimension (North-South)
    depth: float            # Y dimension (East-West)
    height: float           # Z dimension (Up)
    color: Color = Color.GRAY

    def get_corners(self) -> List[LocalCoord]:
        """Get 8 corners of the box."""
        hw, hd, hh = self.width / 2, self.depth / 2, self.height / 2
        cx, cy, cz = self.center.x, self.center.y, self.center.z

        return [
            # Bottom face
            LocalCoord(cx - hw, cy - hd, cz),
            LocalCoord(cx + hw, cy - hd, cz),
            LocalCoord(cx + hw, cy + hd, cz),
            LocalCoord(cx - hw, cy + hd, cz),
            # Top face
            LocalCoord(cx - hw, cy - hd, cz + self.height),
            LocalCoord(cx + hw, cy - hd, cz + self.height),
            LocalCoord(cx + hw, cy + hd, cz + self.height),
            LocalCoord(cx - hw, cy + hd, cz + self.height),
        ]

    def get_faces(self) -> dict:
        """Get face definitions (for target placement)."""
        hw, hd = self.width / 2, self.depth / 2
        cx, cy, cz = self.center.x, self.center.y, self.center.z

        return {
            "north": {  # +X face
                "center": LocalCoord(cx + hw, cy, cz + self.height / 2),
                "normal": (1, 0, 0),
                "width": self.depth,
                "height": self.height
            },
            "south": {  # -X face
                "center": LocalCoord(cx - hw, cy, cz + self.height / 2),
                "normal": (-1, 0, 0),
                "width": self.depth,
                "height": self.height
            },
            "east": {  # +Y face
                "center": LocalCoord(cx, cy + hd, cz + self.height / 2),
                "normal": (0, 1, 0),
                "width": self.width,
                "height": self.height
            },
            "west": {  # -Y face
                "center": LocalCoord(cx, cy - hd, cz + self.height / 2),
                "normal": (0, -1, 0),
                "width": self.width,
                "height": self.height
            },
            "top": {  # +Z face (roof)
                "center": LocalCoord(cx, cy, cz + self.height),
                "normal": (0, 0, 1),
                "width": self.width,
                "height": self.depth
            },
            "bottom": {  # -Z face
                "center": LocalCoord(cx, cy, cz),
                "normal": (0, 0, -1),
                "width": self.width,
                "height": self.depth
            }
        }


@dataclass
class CircleTarget:
    """Circular target (can be on ground or building surface)."""
    position: LocalCoord     # Center position
    radius: float            # Radius in meters
    color: Color
    surface_normal: Tuple[float, float, float] = (0, 0, 1)  # Default: horizontal (facing up)

    def is_on_ground(self) -> bool:
        """Check if target is on ground (z ≈ 0 and normal facing up)."""
        return abs(self.position.z) < 0.5 and self.surface_normal == (0, 0, 1)

    def is_on_wall(self) -> bool:
        """Check if target is on vertical wall."""
        return abs(self.surface_normal[2]) < 0.1  # Normal is mostly horizontal

    def is_on_roof(self) -> bool:
        """Check if target is on roof (z > ground and normal facing up)."""
        return self.position.z > 0.5 and self.surface_normal == (0, 0, 1)


@dataclass
class World:
    """Complete simulated world."""
    ground: Ground = field(default_factory=Ground)
    boxes: List[Box] = field(default_factory=list)
    targets: List[CircleTarget] = field(default_factory=list)

    def add_box(self, center: LocalCoord, width: float, depth: float,
                height: float, color: Color = Color.GRAY) -> Box:
        """Add a rectangular building to the world."""
        box = Box(center, width, depth, height, color)
        self.boxes.append(box)
        return box

    def add_target_on_ground(self, x: float, y: float, radius: float,
                            color: Color) -> CircleTarget:
        """Add a target on the ground plane."""
        target = CircleTarget(
            position=LocalCoord(x, y, self.ground.altitude + 0.01),
            radius=radius,
            color=color,
            surface_normal=(0, 0, 1)  # Facing up
        )
        self.targets.append(target)
        return target

    def add_target_on_box_face(self, box: Box, face: Literal["north", "south", "east", "west", "top"],
                               offset_x: float, offset_y: float, radius: float,
                               color: Color) -> CircleTarget:
        """
        Add a target on a box face.

        Args:
            box: The box to place target on
            face: Which face ("north", "south", "east", "west", "top")
            offset_x: Horizontal offset from face center (meters, -width/2 to +width/2)
            offset_y: Vertical offset from face center (meters, -height/2 to +height/2)
            radius: Target radius
            color: Target color

        Returns:
            Created target
        """
        faces = box.get_faces()
        if face not in faces:
            raise ValueError(f"Invalid face: {face}")

        face_info = faces[face]
        face_center = face_info["center"]
        normal = face_info["normal"]

        # Calculate position based on face orientation
        if face in ["north", "south"]:  # X-aligned faces
            position = LocalCoord(
                x=face_center.x,
                y=face_center.y + offset_x,
                z=face_center.z + offset_y
            )
        elif face in ["east", "west"]:  # Y-aligned faces
            position = LocalCoord(
                x=face_center.x + offset_x,
                y=face_center.y,
                z=face_center.z + offset_y
            )
        else:  # top/bottom (horizontal)
            position = LocalCoord(
                x=face_center.x + offset_x,
                y=face_center.y + offset_y,
                z=face_center.z
            )

        target = CircleTarget(
            position=position,
            radius=radius,
            color=color,
            surface_normal=normal
        )
        self.targets.append(target)
        return target

    def get_all_targets(self) -> List[CircleTarget]:
        """Get all targets in the world."""
        return self.targets

    def get_all_boxes(self) -> List[Box]:
        """Get all boxes in the world."""
        return self.boxes


def create_demo_world() -> World:
    """
    Create a demo world with a building and various targets.

    World layout (top-down view, origin at 0,0):

        Y (East) →
    X   ┌─────────────┐
    (N) │   BUILDING  │  (10m x 8m x 12m high)
    │   │  (Red wall) │
    ↓   │ (Blue roof) │
        └─────────────┘

        (Green ground)

    Coordinates:
    - Building center: (20, 15, 6) - 20m north, 15m east, 6m up (center of 12m tall building)
    - Ground at z=0
    """
    world = World()

    # Add building at (20, 15) with 10m width, 8m depth, 12m height
    building = world.add_box(
        center=LocalCoord(20, 15, 6),  # Center is at half-height
        width=10,   # North-South
        depth=8,    # East-West
        height=12,  # Height
        color=Color.GRAY
    )

    # Add red target on north wall (facing +X direction)
    world.add_target_on_box_face(
        box=building,
        face="north",
        offset_x=0,    # Centered horizontally
        offset_y=2,    # 2m above center (8m from ground)
        radius=0.5,
        color=Color.RED
    )

    # Add blue target on roof
    world.add_target_on_box_face(
        box=building,
        face="top",
        offset_x=2,    # 2m north of roof center
        offset_y=-1,   # 1m west of roof center
        radius=0.4,
        color=Color.BLUE
    )

    # Add green target on ground (south of building)
    world.add_target_on_ground(
        x=10,   # 10m south of building center
        y=15,   # Same east position as building
        radius=0.6,
        color=Color.GREEN
    )

    # Add yellow target on east wall
    world.add_target_on_box_face(
        box=building,
        face="east",
        offset_x=0,
        offset_y=-2,   # 2m below center (4m from ground)
        radius=0.3,
        color=Color.YELLOW
    )

    return world
