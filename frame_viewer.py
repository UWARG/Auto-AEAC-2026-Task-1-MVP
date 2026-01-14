"""
Frame viewer for testing camera target detection.

Loads frames from a pickle file and allows scrolling through them
with left/right arrow keys. Lazily performs detection on the current frame
and displays crosshair + detected colour.

Click anywhere on the frame to see the HSV value at that pixel.
"""

import pickle
import cv2
import sys

sys.path.insert(0, "airside")
from camera import Camera

# Global state for mouse callback
viewer_state = {
    "current_frame": None,
    "hsv_info": None,  # (x, y, h, s, v) or None
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


def draw_crosshair(frame, x: int, y: int, size: int = 20, thickness: int = 2):
    """Draw a crosshair at the specified position."""
    color = (0, 255, 0)  # Green crosshair
    cv2.line(frame, (x - size, y), (x + size, y), color, thickness)
    cv2.line(frame, (x, y - size), (x, y + size), color, thickness)
    cv2.circle(frame, (x, y), 5, color, -1)


def draw_text_box(frame, text: str, position: tuple[int, int] = (10, 30)):
    """Draw a text box with background."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.8
    thickness = 2

    (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)

    x, y = position
    padding = 10

    cv2.rectangle(
        frame,
        (x - padding, y - text_height - padding),
        (x + text_width + padding, y + baseline + padding),
        (0, 0, 0),
        -1
    )
    cv2.putText(frame, text, (x, y), font, font_scale, (255, 255, 255), thickness)


def main():
    # Load frames from pickle file
    print("Loading frames from 'frames' pickle file...")
    try:
        with open("frames", "rb") as f:
            frames = pickle.load(f)
    except FileNotFoundError:
        print("Error: 'frames' file not found")
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

    # Cache for detection results: {frame_index: (colour, center)}
    detection_cache = {}

    current_index = 0

    def get_detection(index: int):
        """Lazily get detection for a frame, using cache if available."""
        if index in detection_cache:
            return detection_cache[index]

        frame = frames[index]
        colour = camera.colour_in_frame(frame)
        center = None
        if colour is not None:
            center = camera.center_of_target_in_frame(frame, colour)

        detection_cache[index] = (colour, center)
        return colour, center

    window_name = "Frame Viewer - Left/Right to navigate, Click for HSV, Q to quit"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window_name, mouse_callback)

    while True:
        # Get current frame and detection
        original_frame = frames[current_index]
        viewer_state["current_frame"] = original_frame  # For mouse callback
        frame = original_frame.copy()
        colour, center = get_detection(current_index)

        # Draw detection info
        if colour is not None and center is not None:
            draw_crosshair(frame, center[0], center[1])
            draw_text_box(frame, f"Detected: {colour.name}", (10, 30))
        else:
            draw_text_box(frame, "NO DETECTION", (10, 30))

        # Draw frame counter
        counter_text = f"Frame {current_index + 1}/{len(frames)}"
        draw_text_box(frame, counter_text, (10, frame.shape[0] - 20))

        # Draw HSV info if a pixel was clicked
        if viewer_state["hsv_info"] is not None:
            px, py, h, s, v = viewer_state["hsv_info"]
            hsv_text = f"HSV at ({px},{py}): H={h} S={s} V={v}"
            draw_text_box(frame, hsv_text, (10, 70))
            # Draw small marker at clicked position
            cv2.circle(frame, (px, py), 3, (255, 0, 255), -1)

        cv2.imshow(window_name, frame)

        key = cv2.waitKey(50) & 0xFF  # 50ms timeout to allow mouse click updates

        if key == 255:  # No key pressed (timeout)
            continue
        elif key == ord('q') or key == 27:  # Q or ESC to quit
            break
        elif key == 81 or key == 2:  # Left arrow
            current_index = (current_index - 1) % len(frames)
        elif key == 83 or key == 3:  # Right arrow
            current_index = (current_index + 1) % len(frames)
        elif key == ord('a'):  # A as alternative left
            current_index = (current_index - 1) % len(frames)
        elif key == ord('d'):  # D as alternative right
            current_index = (current_index + 1) % len(frames)
        elif key == ord('c'):  # C to clear HSV marker
            viewer_state["hsv_info"] = None

    cv2.destroyAllWindows()
    print("Viewer closed")


if __name__ == "__main__":
    main()
