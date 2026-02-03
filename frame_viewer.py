"""
Frame viewer for testing camera target detection.

Loads frames from a pickle file and allows scrolling through them
with left/right arrow keys. Lazily performs detection on the current frame
and displays crosshair + detected colour.

Click anywhere on the frame to see the HSV value at that pixel.
Use up/down arrows to scroll through multiple detections per frame.
"""

import pickle
import cv2
import numpy as np
import sys

sys.path.insert(0, "airside")
from camera import Camera, TargetDetection

# Global state for mouse callback
viewer_state = {
    "current_frame": None,
    "hsv_info": None,  # (x, y, h, s, v) or None
    "show_contour": False,  # Toggle contour view with 't'
}


def mouse_callback(event, x, y, flags, param):
    """Handle mouse clicks to show HSV value at clicked position."""
    if event == cv2.EVENT_LBUTTONDOWN:
        frame = viewer_state["current_frame"]
        if frame is not None and 0 <= y < frame.shape[0] and 0 <= x < frame.shape[1]:
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            h, s, v = hsv_frame[y, x]
            viewer_state["hsv_info"] = (x, y, int(h), int(s), int(v))
            print(f"Clicked ({x}, {y}) -> HSV: ({h}, {s}, {v})")


def draw_crosshair(
    frame, x: int, y: int, color=(0, 255, 0), size: int = 20, thickness: int = 2
):
    """Draw a crosshair at the specified position."""
    cv2.line(frame, (x - size, y), (x + size, y), color, thickness)
    cv2.line(frame, (x, y - size), (x, y + size), color, thickness)
    cv2.circle(frame, (x, y), 5, color, -1)


def draw_text_box(
    frame, text: str, position: tuple[int, int] = (10, 30), font_scale: float = 0.6
):
    """Draw a text box with background."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = 2

    (text_width, text_height), baseline = cv2.getTextSize(
        text, font, font_scale, thickness
    )

    x, y = position
    padding = 5

    cv2.rectangle(
        frame,
        (x - padding, y - text_height - padding),
        (x + text_width + padding, y + baseline + padding),
        (0, 0, 0),
        -1,
    )
    cv2.putText(frame, text, (x, y), font, font_scale, (255, 255, 255), thickness)


def main():
    # Load frames from pickle file
    if len(sys.argv) < 2:
        print("Usage: python frame_viewer.py <frames_file>")
        return
    frames_path = sys.argv[1]
    print(f"Loading frames from '{frames_path}'...")
    try:
        with open(frames_path, "rb") as f:
            frames = pickle.load(f)
    except FileNotFoundError:
        print(f"Error: '{frames_path}' not found")
        return
    except Exception as e:
        print(f"Error loading frames: {e}")
        return

    if not frames:
        print("No frames found in pickle file")
        return

    print(f"Loaded {len(frames)} frames")

    # Create dummy camera for detection functions
    camera = Camera(mode="dummy")

    # Cache for detection results: {frame_index: list[TargetDetection]}
    detection_cache = {}

    current_frame_index = 0
    current_detection_index = 0

    def get_detections(index: int) -> list[TargetDetection]:
        """Lazily get detections for a frame, using cache if available."""
        if index in detection_cache:
            return detection_cache[index]

        frame = frames[index]
        detections = camera.find_targets(frame)
        # Filter out tiny detections
        detections = [d for d in detections if d.area >= 200 and d.circularity > 0.4]
        detection_cache[index] = detections
        return detections

    window_name = "Frame Viewer - Left/Right: frames, Up/Down: detections, T: contour, Click: HSV, Q: quit"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window_name, mouse_callback)

    prev_frame_index = -1

    while True:
        # Reset detection index when frame changes
        if current_frame_index != prev_frame_index:
            current_detection_index = 0
            prev_frame_index = current_frame_index

        # Get current frame and detections
        original_frame = frames[current_frame_index]
        viewer_state["current_frame"] = original_frame  # For mouse callback
        frame = original_frame.copy()
        detections = get_detections(current_frame_index)

        # Draw all detections as small markers
        for i, det in enumerate(detections):
            is_valid = camera.is_valid_target(det)
            marker_color = (
                (0, 255, 0) if is_valid else (0, 0, 255)
            )  # Green if valid, red if not
            cv2.circle(frame, (int(det.x), int(det.y)), 8, marker_color, 2)

        # Draw info for selected detection
        if detections:
            # Clamp detection index
            current_detection_index = current_detection_index % len(detections)
            det = detections[current_detection_index]
            is_valid = camera.is_valid_target(det)

            # Draw crosshair on selected detection
            crosshair_color = (0, 255, 0) if is_valid else (0, 0, 255)
            draw_crosshair(frame, int(det.x), int(det.y), color=crosshair_color)

            # Draw detection info
            y_offset = 30
            valid_str = "VALID" if is_valid else "INVALID"
            draw_text_box(
                frame,
                f"Detection {current_detection_index + 1}/{len(detections)} [{valid_str}]",
                (10, y_offset),
            )
            y_offset += 30
            draw_text_box(frame, f"Colour: {det.colour.name}", (10, y_offset))
            y_offset += 30
            draw_text_box(
                frame, f"Position: ({det.x:.1f}, {det.y:.1f})", (10, y_offset)
            )
            y_offset += 30
            draw_text_box(frame, f"Area: {det.area:.1f} px", (10, y_offset))
            y_offset += 30
            draw_text_box(frame, f"Circularity: {det.circularity:.3f}", (10, y_offset))
            y_offset += 30
            draw_text_box(frame, f"Coverage: {det.coverage:.3f}", (10, y_offset))
        else:
            draw_text_box(frame, "NO DETECTIONS", (10, 30))

        # Draw frame counter
        counter_text = f"Frame {current_frame_index + 1}/{len(frames)}"
        draw_text_box(frame, counter_text, (10, frame.shape[0] - 20))

        # Draw HSV info if a pixel was clicked
        if viewer_state["hsv_info"] is not None:
            px, py, h, s, v = viewer_state["hsv_info"]
            hsv_text = f"HSV at ({px},{py}): H={h} S={s} V={v}"
            draw_text_box(frame, hsv_text, (frame.shape[1] - 380, 30))
            # Draw small marker at clicked position
            cv2.circle(frame, (px, py), 3, (255, 0, 255), -1)

        # Show contour view if toggled and there's a selected detection with contour
        if viewer_state["show_contour"] and detections:
            det = detections[current_detection_index]
            if det.contour is not None:
                # Create black and white contour image
                contour_img = np.zeros(original_frame.shape[:2], dtype=np.uint8)
                cv2.drawContours(contour_img, [det.contour], -1, 255, -1)
                # Convert to BGR for display
                contour_display = cv2.cvtColor(contour_img, cv2.COLOR_GRAY2BGR)
                draw_text_box(
                    contour_display, f"Contour View - {det.colour.name}", (10, 30)
                )
                cv2.imshow(window_name, contour_display)
            else:
                cv2.imshow(window_name, frame)
        else:
            cv2.imshow(window_name, frame)

        key = cv2.waitKey(50) & 0xFF  # 50ms timeout to allow mouse click updates

        if key == 255:  # No key pressed (timeout)
            continue
        elif key == ord("q") or key == 27:  # Q or ESC to quit
            break
        elif key == 81 or key == 2 or key == ord("a"):  # Left arrow or A
            current_frame_index = (current_frame_index - 1) % len(frames)
        elif key == 83 or key == 3 or key == ord("d"):  # Right arrow or D
            current_frame_index = (current_frame_index + 1) % len(frames)
        elif key == 82 or key == 0 or key == ord("w"):  # Up arrow or W
            if detections:
                current_detection_index = (current_detection_index - 1) % len(
                    detections
                )
        elif key == 84 or key == 1 or key == ord("s"):  # Down arrow or S
            if detections:
                current_detection_index = (current_detection_index + 1) % len(
                    detections
                )
        elif key == ord("c"):  # C to clear HSV marker
            viewer_state["hsv_info"] = None
        elif key == ord("t"):  # T to toggle contour view
            viewer_state["show_contour"] = not viewer_state["show_contour"]

    cv2.destroyAllWindows()
    print("Viewer closed")


if __name__ == "__main__":
    main()
