"""
Test script to visualize color detection algorithm.
Shows detection masks, contours, and final circular target detection.
"""

import cv2
import numpy as np
import sys
import os
from pathlib import Path

# Add parent directory to path to import util
sys.path.append(str(Path(__file__).parent.parent))
from util import Colour, Colours


def visualize_color_detection(frame: np.ndarray, image_name: str):
    """
    Visualize the color detection process step by step.
    """
    if frame is None or frame.size == 0:
        print("Invalid frame")
        return

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_height, frame_width = frame.shape[:2]
    frame_center_x = frame_width / 2
    frame_center_y = frame_height / 2

    # Thresholds for detection
    MIN_AREA = 100  # Minimum contour area in pixels
    MIN_CIRCULARITY = 0.6  # Circularity threshold (1.0 = perfect circle)
    MIN_FILL_RATIO = (
        0.7  # Minimum ratio of colored pixels to contour area (1.0 = completely filled)
    )

    # Track the closest circular target to frame center
    min_distance = float("inf")
    detected_colour = None
    detected_center = None

    # Create a figure to show all masks
    all_masks = []
    all_contour_images = []

    print(f"\n{'='*60}")
    print(f"Processing: {image_name}")
    print(f"Frame size: {frame_width}x{frame_height}")
    print(f"Frame center: ({frame_center_x:.1f}, {frame_center_y:.1f})")
    print(f"{'='*60}\n")

    for colour in [c.value for c in Colours]:
        print(f"Checking {colour.name}:")
        print(f"  HSV range: {colour.lower_hsv} to {colour.upper_hsv}")

        mask = cv2.inRange(frame_hsv, colour.lower_hsv, colour.upper_hsv)
        pixel_count = cv2.countNonZero(mask)
        print(f"  Total pixels matching: {pixel_count}")

        # Create colored mask for visualization
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.putText(
            mask_colored,
            f"{colour.name}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2,
        )
        all_masks.append(mask_colored)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(f"  Contours found: {len(contours)}")

        # Draw contours on a copy of the frame
        contour_image = frame.copy()
        cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 2)

        valid_contours = 0
        for idx, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area < MIN_AREA:
                continue

            # Calculate circularity: 4*pi*area / perimeter^2
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue

            circularity = 4 * np.pi * area / (perimeter * perimeter)

            # Check fill ratio: ensure the contour is solid, not hollow
            contour_mask = np.zeros(frame_hsv.shape[:2], dtype=np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, -1)
            colored_pixels_in_contour = cv2.countNonZero(
                cv2.bitwise_and(mask, contour_mask)
            )
            fill_ratio = colored_pixels_in_contour / area if area > 0 else 0

            # Calculate center
            moments = cv2.moments(contour)
            if moments["m00"] != 0:
                center_x = moments["m10"] / moments["m00"]
                center_y = moments["m01"] / moments["m00"]

                # Calculate distance from frame center
                distance = np.sqrt(
                    (center_x - frame_center_x) ** 2 + (center_y - frame_center_y) ** 2
                )

                print(
                    f"    Contour {idx}: area={area:.0f}, circularity={circularity:.3f}, "
                    f"fill_ratio={fill_ratio:.3f}, "
                    f"center=({center_x:.1f}, {center_y:.1f}), distance={distance:.1f}"
                )

                # Draw center point
                cv2.circle(
                    contour_image, (int(center_x), int(center_y)), 5, (255, 0, 0), -1
                )

                # Check if this is a valid circular contour
                if circularity >= MIN_CIRCULARITY and fill_ratio >= MIN_FILL_RATIO:
                    valid_contours += 1
                    # Draw circle around valid circular contours
                    cv2.circle(
                        contour_image,
                        (int(center_x), int(center_y)),
                        10,
                        (0, 255, 255),
                        2,
                    )

                    if distance < min_distance:
                        min_distance = distance
                        detected_colour = colour
                        detected_center = (int(center_x), int(center_y))

        print(f"  Valid circular contours: {valid_contours}\n")

        cv2.putText(
            contour_image,
            f"{colour.name} - {valid_contours} valid",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )
        all_contour_images.append(contour_image)

    # Create final result image
    result_image = frame.copy()
    cv2.circle(
        result_image, (int(frame_center_x), int(frame_center_y)), 10, (255, 255, 255), 2
    )
    cv2.line(
        result_image,
        (int(frame_center_x) - 20, int(frame_center_y)),
        (int(frame_center_x) + 20, int(frame_center_y)),
        (255, 255, 255),
        2,
    )
    cv2.line(
        result_image,
        (int(frame_center_x), int(frame_center_y) - 20),
        (int(frame_center_x), int(frame_center_y) + 20),
        (255, 255, 255),
        2,
    )

    if detected_colour is not None:
        print(f"{'='*60}")
        print(
            f"DETECTED: {detected_colour.name} at distance {min_distance:.1f} pixels from center"
        )
        print(f"Target center: {detected_center}")
        print(f"{'='*60}\n")

        # Draw detected target
        cv2.circle(result_image, detected_center, 15, (0, 255, 0), 3)
        cv2.line(
            result_image,
            (int(frame_center_x), int(frame_center_y)),
            detected_center,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            result_image,
            f"DETECTED: {detected_colour.name}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )
    else:
        print(f"{'='*60}")
        print("NO CIRCULAR TARGET DETECTED")
        print(f"{'='*60}\n")
        cv2.putText(
            result_image,
            "NO TARGET DETECTED",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2,
        )

    # Stack masks horizontally
    masks_row = np.hstack(all_masks)

    # Stack contour images horizontally (might need to split into rows if too many colors)
    contours_row = np.hstack(all_contour_images)

    # Show all visualizations
    cv2.imshow(f"{image_name} - Masks", masks_row)
    cv2.imshow(f"{image_name} - Contours", contours_row)
    cv2.imshow(f"{image_name} - Result", result_image)


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Visualize color detection on an image"
    )
    parser.add_argument("image_path", type=str, help="Path to the image file")
    args = parser.parse_args()

    image_path = Path(args.image_path)
    if not image_path.exists():
        print(f"Image not found: {image_path}")
        return

    frame = cv2.imread(str(image_path))
    if frame is None:
        print(f"Failed to load {image_path}")
        return

    visualize_color_detection(frame, image_path.name)

    print("\nPress any key to close all windows...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
