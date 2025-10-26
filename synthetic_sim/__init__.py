"""
Synthetic simulation package for GPS-accurate camera testing.

Provides:
- Coordinate transformations (GPS ↔ Local Cartesian)
- World definition with primitives (ground, boxes, targets)
- Synthetic camera rendering with perspective projection
- Flight demo for testing
"""

from synthetic_sim.coordinates import CoordinateTransform, GPSCoord, LocalCoord
from synthetic_sim.world import World, Box, CircleTarget, Color, create_demo_world
from synthetic_sim.camera import SyntheticCamera, CameraConfig

__all__ = [
    'CoordinateTransform',
    'GPSCoord',
    'LocalCoord',
    'World',
    'Box',
    'CircleTarget',
    'Color',
    'create_demo_world',
    'SyntheticCamera',
    'CameraConfig',
]
