"""
Target Detector Module
Handles colored target detection for the drone competition
Detects black, white, red, yellow, blue, and green circular targets
"""

import cv2
import numpy as np
import logging
from typing import Optional, Tuple, List, Dict
from dataclasses import dataclass
from config import TARGET_COLORS, MIN_TARGET_AREA, MAX_TARGET_AREA

logger = logging.getLogger(__name__)

@dataclass
class Target:
    """Represents a detected target"""
    center_x: int
    center_y: int
    radius: int
    color: str
    area: float
    confidence: float  # 0-1 confidence score
    
    def to_dict(self) -> Dict:
        return {
            'center_x': self.center_x,
            'center_y': self.center_y,
            'radius': self.radius,
            'color': self.color,
            'area': self.area,
            'confidence': self.confidence
        }

class TargetDetector:
    """
    Detects colored circular targets in camera frames
    Supports multiple target colors and provides confidence scoring
    """
    
    def __init__(self):
        self.target_colors = TARGET_COLORS
        self.min_area = MIN_TARGET_AREA
        self.max_area = MAX_TARGET_AREA
        
        # Detection parameters
        self.gaussian_blur_kernel = (5, 5)
        self.morphology_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.min_circularity = 0.7  # Minimum circularity for target validation
        self.min_contour_points = 10  # Minimum points in contour
        
        # Hough circle parameters
        self.hough_dp = 1
        self.hough_min_dist = 50
        self.hough_param1 = 50
        self.hough_param2 = 30
        self.hough_min_radius = 10
        self.hough_max_radius = 200
        
    def detect_targets(self, frame) -> List[Target]:
        """
        Detect all targets in the frame
        Returns list of detected targets sorted by confidence
        """
        if frame is None:
            return []
        
        targets = []
        
        # Convert to HSV for color detection
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Apply Gaussian blur to reduce noise
        hsv_frame = cv2.GaussianBlur(hsv_frame, self.gaussian_blur_kernel, 0)
        
        # Detect targets for each color
        for color_name, color_range in self.target_colors.items():
            if color_name == 'red_upper':  # Skip red_upper as it's handled with red
                continue
                
            color_targets = self._detect_color_targets(hsv_frame, color_name)
            targets.extend(color_targets)
        
        # Remove duplicate detections (same position, different colors)
        targets = self._remove_duplicate_targets(targets)
        
        # Sort by confidence (highest first)
        targets.sort(key=lambda t: t.confidence, reverse=True)
        
        return targets
    
    def _detect_color_targets(self, hsv_frame, color_name: str) -> List[Target]:
        """Detect targets of a specific color"""
        targets = []
        
        try:
            # Create color mask
            if color_name == 'red':
                # Red needs special handling as it wraps around in HSV
                mask1 = cv2.inRange(hsv_frame, self.target_colors['red']['lower'], 
                                   self.target_colors['red']['upper'])
                mask2 = cv2.inRange(hsv_frame, self.target_colors['red_upper']['lower'], 
                                   self.target_colors['red_upper']['upper'])
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv_frame, self.target_colors[color_name]['lower'], 
                                 self.target_colors[color_name]['upper'])
            
            # Clean up the mask
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morphology_kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morphology_kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Process each contour
            for contour in contours:
                target = self._process_contour(contour, color_name)
                if target:
                    targets.append(target)
            
            # Also try Hough circle detection for additional validation
            circles = cv2.HoughCircles(
                mask, cv2.HOUGH_GRADIENT, self.hough_dp, self.hough_min_dist,
                param1=self.hough_param1, param2=self.hough_param2,
                minRadius=self.hough_min_radius, maxRadius=self.hough_max_radius
            )
            
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                for (x, y, r) in circles:
                    # Create target from circle detection
                    area = np.pi * r * r
                    if self.min_area <= area <= self.max_area:
                        confidence = self._calculate_circle_confidence(mask, x, y, r)
                        if confidence > 0.5:  # Only add high-confidence circles
                            target = Target(
                                center_x=x, center_y=y, radius=r,
                                color=color_name, area=area, confidence=confidence
                            )
                            targets.append(target)
        
        except Exception as e:
            logger.error(f"Error detecting {color_name} targets: {e}")
        
        return targets
    
    def _process_contour(self, contour, color_name: str) -> Optional[Target]:
        """Process a contour to determine if it's a valid target"""
        try:
            # Check if contour has enough points
            if len(contour) < self.min_contour_points:
                return None
            
            # Calculate contour properties
            area = cv2.contourArea(contour)
            
            # Check area constraints
            if area < self.min_area or area > self.max_area:
                return None
            
            # Calculate circularity
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                return None
            
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            
            # Check circularity constraint
            if circularity < self.min_circularity:
                return None
            
            # Calculate center and radius
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                return None
            
            center_x = int(moments["m10"] / moments["m00"])
            center_y = int(moments["m01"] / moments["m00"])
            
            # Calculate equivalent radius
            radius = int(np.sqrt(area / np.pi))
            
            # Calculate confidence based on circularity and area
            confidence = min(1.0, circularity * (area / self.max_area) * 2)
            
            return Target(
                center_x=center_x, center_y=center_y, radius=radius,
                color=color_name, area=area, confidence=confidence
            )
            
        except Exception as e:
            logger.error(f"Error processing contour for {color_name}: {e}")
            return None
    
    def _calculate_circle_confidence(self, mask, x: int, y: int, r: int) -> float:
        """Calculate confidence for a detected circle"""
        try:
            # Create a circular region
            circle_mask = np.zeros_like(mask)
            cv2.circle(circle_mask, (x, y), r, 255, -1)
            
            # Calculate what percentage of the circle is filled with the target color
            intersection = cv2.bitwise_and(mask, circle_mask)
            fill_ratio = np.sum(intersection > 0) / np.sum(circle_mask > 0)
            
            return min(1.0, fill_ratio * 1.2)  # Boost confidence slightly
            
        except Exception:
            return 0.5  # Default confidence
    
    def _remove_duplicate_targets(self, targets: List[Target]) -> List[Target]:
        """Remove duplicate targets that are too close to each other"""
        if len(targets) <= 1:
            return targets
        
        # Sort targets by confidence (highest first)
        targets.sort(key=lambda t: t.confidence, reverse=True)
        
        filtered_targets = []
        min_distance = 50  # Minimum distance between target centers
        
        for target in targets:
            is_duplicate = False
            
            for existing_target in filtered_targets:
                distance = np.sqrt((target.center_x - existing_target.center_x) ** 2 + 
                                 (target.center_y - existing_target.center_y) ** 2)
                
                if distance < min_distance:
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                filtered_targets.append(target)
        
        return filtered_targets
    
    def get_best_target(self, targets: List[Target]) -> Optional[Target]:
        """Get the best target from a list (highest confidence)"""
        if not targets:
            return None
        
        return max(targets, key=lambda t: t.confidence)
    
    def draw_targets(self, frame, targets: List[Target], 
                    highlight_best: bool = True) -> any:
        """Draw detected targets on the frame"""
        if frame is None or not targets:
            return frame
        
        best_target = self.get_best_target(targets) if highlight_best else None
        
        for i, target in enumerate(targets):
            color = (0, 0, 255) if target == best_target else (0, 255, 0)
            thickness = 3 if target == best_target else 2
            
            # Draw circle around target
            cv2.circle(frame, (target.center_x, target.center_y), 
                      target.radius + 5, color, thickness)
            
            # Draw center point
            cv2.circle(frame, (target.center_x, target.center_y), 5, color, -1)
            
            # Draw crosshairs
            cv2.line(frame, (target.center_x - 20, target.center_y), 
                    (target.center_x + 20, target.center_y), color, thickness)
            cv2.line(frame, (target.center_x, target.center_y - 20), 
                    (target.center_x, target.center_y + 20), color, thickness)
            
            # Draw label
            label = f"{target.color} ({target.confidence:.2f})"
            cv2.putText(frame, label, (target.center_x - 40, target.center_y - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        return frame
    
    def calculate_target_offset(self, target: Target, frame_width: int, 
                              frame_height: int) -> Tuple[int, int]:
        """Calculate offset of target from frame center"""
        center_x = frame_width // 2
        center_y = frame_height // 2
        
        offset_x = target.center_x - center_x
        offset_y = target.center_y - center_y
        
        return offset_x, offset_y
    
    def update_detection_parameters(self, **kwargs):
        """Update detection parameters"""
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
                logger.info(f"Updated {key} to {value}")
            else:
                logger.warning(f"Unknown parameter: {key}")
    
    def get_detection_info(self) -> Dict:
        """Get information about detection parameters"""
        return {
            'target_colors': list(self.target_colors.keys()),
            'min_area': self.min_area,
            'max_area': self.max_area,
            'min_circularity': self.min_circularity,
            'gaussian_blur_kernel': self.gaussian_blur_kernel,
            'hough_parameters': {
                'dp': self.hough_dp,
                'min_dist': self.hough_min_dist,
                'param1': self.hough_param1,
                'param2': self.hough_param2,
                'min_radius': self.hough_min_radius,
                'max_radius': self.hough_max_radius
            }
        }