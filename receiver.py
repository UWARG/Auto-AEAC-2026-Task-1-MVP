import io
import math
import socket
import struct
import threading
import tkinter as tk
from tkinter import messagebox
from typing import Optional, Tuple

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageTk

# Click mode for "Generate output" flow
SELECT_TARGET = "target"
SELECT_REFERENCE = "reference"

# Crosshair colors (match prompt text color)
TARGET_CROSSHAIR_COLOUR = "lime"  # green, for "Select target center"
REFERENCE_CROSSHAIR_COLOUR = "deepskyblue"  # blue, for "Select reference point"
CROSSHAIR_SIZE = 8  # half-length of each crosshair arm in pixels


# =========================
# Configuration
# =========================

# IP and port of the transmitter (Raspberry Pi on the drone)
TRANSMITTER_HOST = "192.168.232.15"
TRANSMITTER_PORT = 5000

SOCKET_TIMEOUT = 10.0  # seconds

# Camera FOV (radians) used to project clicked pixels into metric offsets
CAMERA_HFOV_RAD = 0.64889
CAMERA_VFOV_RAD = 0.41438

# Fisheye correction (only near the edges). Correction = 1 + this * (angle / edge_angle)^2.
# 0.0 = no correction. Use e.g. 0.1–0.3 if the lens compresses angles toward the edges.
FISHEYE_EDGE_FACTOR = 0.0


class ReceiverApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Drone Image Receiver")

        # GUI elements
        self.button = tk.Button(
            root, text="Capture image", command=self.on_capture_clicked
        )
        self.button.pack(pady=8)

        self.image_label = tk.Label(root)
        self.image_label.pack(padx=8, pady=8)

        self.range_label = tk.Label(root, text="Downwards: N/A    Depth(center): N/A")
        self.range_label.pack(pady=(0, 2))
        self.pitch_label = tk.Label(root, text="Pitch: N/A")
        self.pitch_label.pack(pady=(0, 2))
        self.roll_label = tk.Label(root, text="Roll: N/A")
        self.roll_label.pack(pady=(0, 8))
        self.capture_status_label = tk.Label(root, text="", fg="orange red")
        self.capture_status_label.pack(pady=(0, 4))

        # Generate output section
        gen_frame = tk.Frame(root)
        gen_frame.pack(pady=8, padx=8, fill=tk.X)

        tk.Label(gen_frame, text="Mode:").grid(
            row=0, column=0, sticky=tk.W, padx=(0, 4)
        )
        self._mode_var = tk.StringVar(value="aided")
        mode_frame = tk.Frame(gen_frame)
        mode_frame.grid(row=0, column=1, sticky=tk.W, pady=2)
        tk.Radiobutton(
            mode_frame,
            text="Aided",
            variable=self._mode_var,
            value="aided",
            command=self._on_mode_changed,
        ).pack(side=tk.LEFT, padx=(0, 12))
        tk.Radiobutton(
            mode_frame,
            text="Full manual",
            variable=self._mode_var,
            value="full_manual",
            command=self._on_mode_changed,
        ).pack(side=tk.LEFT)

        tk.Label(gen_frame, text="Target colour:").grid(
            row=1, column=0, sticky=tk.W, padx=(0, 4)
        )
        self._colour_entry = tk.Entry(gen_frame, width=20)
        self._colour_entry.grid(row=1, column=1, sticky=tk.W, pady=2)

        tk.Label(gen_frame, text="Reference description:").grid(
            row=2, column=0, sticky=tk.W, padx=(0, 4)
        )
        self._ref_desc_entry = tk.Entry(gen_frame, width=30)
        self._ref_desc_entry.grid(row=2, column=1, sticky=tk.W, pady=2)

        tk.Label(gen_frame, text="Target:").grid(
            row=3, column=0, sticky=tk.W, padx=(0, 4)
        )
        self._target_direction = tk.StringVar(value="forwards")
        target_frame = tk.Frame(gen_frame)
        target_frame.grid(row=3, column=1, sticky=tk.W, pady=2)
        tk.Radiobutton(
            target_frame,
            text="Forwards",
            variable=self._target_direction,
            value="forwards",
        ).pack(side=tk.LEFT, padx=(0, 12))
        self._downwards_radio = tk.Radiobutton(
            target_frame,
            text="Downwards",
            variable=self._target_direction,
            value="downwards",
            state=tk.DISABLED,
        )
        self._downwards_radio.pack(side=tk.LEFT)

        self._gen_prompt_label = tk.Label(gen_frame, text="")
        self._gen_prompt_label.grid(row=4, column=0, columnspan=2, sticky=tk.W, pady=4)

        btn_frame = tk.Frame(gen_frame)
        btn_frame.grid(row=5, column=0, columnspan=2, pady=4)
        self._action_btn = tk.Button(
            btn_frame, text="Generate output", command=self._on_action_clicked
        )
        self._action_btn.pack(side=tk.LEFT, padx=(0, 8))
        tk.Button(btn_frame, text="Clear", command=self._on_clear_clicked).pack(
            side=tk.LEFT
        )

        self._output_text = tk.Text(
            root, height=4, width=60, wrap=tk.WORD, state=tk.DISABLED
        )
        self._output_text.pack(pady=8, padx=8, fill=tk.X)

        # Keep a reference to the last PhotoImage to prevent garbage collection
        self._current_photo: Optional[ImageTk.PhotoImage] = None
        # Base display image (with center dot only) for redrawing with crosshairs
        self._base_display_image: Optional[Image.Image] = None
        self._base_depth_map: Optional[np.ndarray] = None

        # Last telemetry from capture (for generate output)
        self._last_downwards: Optional[float] = None
        self._last_center_depth: Optional[float] = None
        self._last_pitch: Optional[float] = None
        self._last_roll: Optional[float] = None

        # Generate-output click state: None | SELECT_TARGET | SELECT_REFERENCE
        self._click_mode: Optional[str] = None
        self._target_xy: Optional[Tuple[int, int]] = None
        self._ref_xy: Optional[Tuple[int, int]] = None

        # Protect against multiple concurrent capture threads
        self._capture_lock = threading.Lock()

    def on_capture_clicked(self) -> None:
        if not self._capture_lock.acquire(blocking=False):
            # A capture is already in progress
            return

        self.button.config(state=tk.DISABLED)
        self.capture_status_label.config(text="Waiting for level...")

        thread = threading.Thread(target=self._capture_worker, daemon=True)
        thread.start()

    def _capture_worker(self) -> None:
        try:
            downwards_range, center_depth, pitch, roll, image, depth_map = (
                self.request_image()
            )
        except Exception as exc:
            self.root.after(0, self._handle_capture_error, exc)
        else:
            self.root.after(
                0,
                self._handle_capture_success,
                downwards_range,
                center_depth,
                pitch,
                roll,
                image,
                depth_map,
            )
        finally:
            self.root.after(0, self._release_capture_lock)

    def _release_capture_lock(self) -> None:
        self._capture_lock.release()
        self.button.config(state=tk.NORMAL)

    def _handle_capture_error(self, exc: Exception) -> None:
        self.capture_status_label.config(text="")
        messagebox.showerror(
            "Capture failed", f"Failed to capture image from transmitter:\n{exc}"
        )

    def _redraw_image_with_crosshairs(self) -> None:
        """Redraw the image from base, adding target (green) and reference (blue) crosshairs."""
        if self._base_display_image is None:
            return
        img = self._base_display_image.copy()
        draw = ImageDraw.Draw(img)
        s = CROSSHAIR_SIZE
        if self._target_xy is not None:
            tx, ty = self._target_xy
            draw.line((tx - s, ty, tx + s, ty), fill=TARGET_CROSSHAIR_COLOUR, width=2)
            draw.line((tx, ty - s, tx, ty + s), fill=TARGET_CROSSHAIR_COLOUR, width=2)
        if self._ref_xy is not None:
            rx, ry = self._ref_xy
            draw.line(
                (rx - s, ry, rx + s, ry), fill=REFERENCE_CROSSHAIR_COLOUR, width=2
            )
            draw.line(
                (rx, ry - s, rx, ry + s), fill=REFERENCE_CROSSHAIR_COLOUR, width=2
            )
        self._current_photo = ImageTk.PhotoImage(image=img)
        self.image_label.config(image=self._current_photo)

    def _on_mode_changed(self) -> None:
        """Update action button text when Aided / Full manual changes."""
        if self._mode_var.get() == "full_manual":
            self._action_btn.config(text="Select points to measure")
        else:
            self._action_btn.config(text="Generate output")

    def _on_clear_clicked(self) -> None:
        """Clear target and reference so the user can reselect them."""
        self._target_xy = None
        self._ref_xy = None
        self._click_mode = None
        try:
            self.image_label.unbind("<Button-1>")
        except tk.TclError:
            pass
        self._gen_prompt_label.config(text="", fg="black")
        self._redraw_image_with_crosshairs()

    def _on_action_clicked(self) -> None:
        """Handle main button: Aided = generate output flow; Full manual = two-point measure."""
        if self._current_photo is None:
            messagebox.showinfo("No image", "Capture an image first.")
            return
        if self._mode_var.get() == "full_manual":
            self._start_full_manual_flow()
            return
        # Aided mode
        if self._target_xy is not None and self._ref_xy is not None:
            colour = (self._colour_entry.get() or "").strip()
            ref_desc = (self._ref_desc_entry.get() or "").strip()
            if not colour or not ref_desc:
                messagebox.showwarning(
                    "Missing information",
                    "Please fill both Target colour and Reference description, then press Generate output.",
                )
                return
            self._write_output(colour, ref_desc)
            return
        self._click_mode = SELECT_TARGET
        self._target_xy = None
        self._ref_xy = None
        self._gen_prompt_label.config(
            text="Select target center", fg=TARGET_CROSSHAIR_COLOUR
        )
        self._redraw_image_with_crosshairs()
        self.image_label.bind("<Button-1>", self._on_image_click)

    def _start_full_manual_flow(self) -> None:
        """Start two-point selection for full manual; distances drawn on image."""
        self._click_mode = SELECT_TARGET
        self._target_xy = None
        self._ref_xy = None
        self._gen_prompt_label.config(
            text="Select first point", fg=TARGET_CROSSHAIR_COLOUR
        )
        self._redraw_image_with_crosshairs()
        self.image_label.bind("<Button-1>", self._on_image_click)

    @staticmethod
    def _rotate_depth_map(depth_map: np.ndarray, angle_deg: float) -> np.ndarray:
        height, width = depth_map.shape[:2]
        center = (width / 2.0, height / 2.0)
        matrix = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
        cos = abs(matrix[0, 0])
        sin = abs(matrix[0, 1])

        new_width = int((height * sin) + (width * cos))
        new_height = int((height * cos) + (width * sin))
        matrix[0, 2] += (new_width / 2) - center[0]
        matrix[1, 2] += (new_height / 2) - center[1]

        return cv2.warpAffine(
            depth_map,
            matrix,
            (new_width, new_height),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )

    def _sample_depth_m(self, x: int, y: int, radius: int = 4) -> Optional[float]:
        if self._base_depth_map is None:
            return None
        height, width = self._base_depth_map.shape[:2]
        if x < 0 or y < 0 or x >= width or y >= height:
            return None
        x0 = max(0, x - radius)
        x1 = min(width, x + radius + 1)
        y0 = max(0, y - radius)
        y1 = min(height, y + radius + 1)
        patch = self._base_depth_map[y0:y1, x0:x1]
        valid = patch[patch > 0]
        if valid.size == 0:
            return None
        return float(np.median(valid) / 1000.0)

    def _compute_corrected_up_m(self) -> Optional[float]:
        """
        Corrected 'up' distance (m): downward rangefinder minus the vertical offset of the
        target from the drone, using the target's sampled OAK-D depth and the vertical FOV.
        Returns None if readings or image are missing/invalid.
        """
        if self._base_display_image is None or self._target_xy is None:
            return None
        d_down = self._last_downwards
        tx, ty = self._target_xy
        target_depth = self._sample_depth_m(tx, ty)
        if d_down is None or target_depth is None:
            return None
        if d_down != d_down or target_depth != target_depth:  # NaN
            return None
        H = self._base_display_image.height
        # Vertical angle from image center (rad): positive when target below center
        half_vfov = CAMERA_VFOV_RAD / 2
        angle_v = (ty - H / 2) / (H / 2) * half_vfov
        angle_v = angle_v * (1.0 + FISHEYE_EDGE_FACTOR * (angle_v / half_vfov) ** 2)
        delta_h = target_depth * math.tan(angle_v)
        return d_down - delta_h

    def _compute_lateral_offset_m(self) -> Optional[float]:
        """
        Horizontal distance (m) of the target from the reference, using each point's
        sampled OAK-D depth and the horizontal FOV. Positive = target to the right
        of reference in image.
        Returns None if cannot compute.
        """
        if (
            self._base_display_image is None
            or self._target_xy is None
            or self._ref_xy is None
        ):
            return None
        W = self._base_display_image.width
        tx, ty = self._target_xy
        rx, ry = self._ref_xy
        target_depth = self._sample_depth_m(tx, ty)
        ref_depth = self._sample_depth_m(rx, ry)
        if target_depth is None or ref_depth is None:
            return None
        half_hfov = CAMERA_HFOV_RAD / 2
        angle_tx = (tx - W / 2) / (W / 2) * half_hfov
        angle_rx = (rx - W / 2) / (W / 2) * half_hfov
        angle_tx = angle_tx * (1.0 + FISHEYE_EDGE_FACTOR * (angle_tx / half_hfov) ** 2)
        angle_rx = angle_rx * (1.0 + FISHEYE_EDGE_FACTOR * (angle_rx / half_hfov) ** 2)
        target_x_m = target_depth * math.tan(angle_tx)
        ref_x_m = ref_depth * math.tan(angle_rx)
        return target_x_m - ref_x_m

    def _compute_delta_up_sideways_m(
        self,
    ) -> Tuple[Optional[float], Optional[float]]:
        """
        Distance up (vertical) and sideways (horizontal) in metres between the two
        selected points (point 1 = target_xy, point 2 = ref_xy). Uses FOV and
        sampled OAK-D depth at each point.
        Returns (delta_up_m, delta_sideways_m); either can be None if unavailable.
        """
        if (
            self._base_display_image is None
            or self._target_xy is None
            or self._ref_xy is None
        ):
            return None, None
        W, H = self._base_display_image.width, self._base_display_image.height
        x1, y1 = self._target_xy
        x2, y2 = self._ref_xy
        depth1 = self._sample_depth_m(x1, y1)
        depth2 = self._sample_depth_m(x2, y2)
        if depth1 is None or depth2 is None:
            return None, None
        half_vfov = CAMERA_VFOV_RAD / 2
        half_hfov = CAMERA_HFOV_RAD / 2
        # Vertical angles (rad)
        a1 = (y1 - H / 2) / (H / 2) * half_vfov
        a2 = (y2 - H / 2) / (H / 2) * half_vfov
        a1 = a1 * (1.0 + FISHEYE_EDGE_FACTOR * (a1 / half_vfov) ** 2)
        a2 = a2 * (1.0 + FISHEYE_EDGE_FACTOR * (a2 / half_vfov) ** 2)
        delta_up = (depth2 * math.tan(a2)) - (depth1 * math.tan(a1))
        # Horizontal angles (rad)
        b1 = (x1 - W / 2) / (W / 2) * half_hfov
        b2 = (x2 - W / 2) / (W / 2) * half_hfov
        b1 = b1 * (1.0 + FISHEYE_EDGE_FACTOR * (b1 / half_hfov) ** 2)
        b2 = b2 * (1.0 + FISHEYE_EDGE_FACTOR * (b2 / half_hfov) ** 2)
        delta_sideways = (depth2 * math.tan(b2)) - (depth1 * math.tan(b1))
        return delta_up, delta_sideways

    def _draw_manual_measurements(self) -> None:
        """Draw both points and the distance up/sideways between them on the image."""
        if self._base_display_image is None:
            return
        delta_up, delta_sideways = self._compute_delta_up_sideways_m()
        img = self._base_display_image.copy()
        draw = ImageDraw.Draw(img)
        s = CROSSHAIR_SIZE
        if self._target_xy is not None:
            tx, ty = self._target_xy
            draw.line((tx - s, ty, tx + s, ty), fill=TARGET_CROSSHAIR_COLOUR, width=2)
            draw.line((tx, ty - s, tx, ty + s), fill=TARGET_CROSSHAIR_COLOUR, width=2)
        if self._ref_xy is not None:
            rx, ry = self._ref_xy
            draw.line(
                (rx - s, ry, rx + s, ry), fill=REFERENCE_CROSSHAIR_COLOUR, width=2
            )
            draw.line(
                (rx, ry - s, rx, ry + s), fill=REFERENCE_CROSSHAIR_COLOUR, width=2
            )
        up_str = f"{delta_up:.2f} m" if delta_up is not None else "N/A"
        side_str = f"{delta_sideways:.2f} m" if delta_sideways is not None else "N/A"
        try:
            font = ImageFont.truetype("arial.ttf", 16)
        except (OSError, ImportError):
            font = ImageFont.load_default()
        draw.text((10, 10), f"Up: {up_str}", fill="white", font=font)
        draw.text((10, 28), f"Sideways: {side_str}", fill="white", font=font)
        self._current_photo = ImageTk.PhotoImage(image=img)
        self.image_label.config(image=self._current_photo)

    def _write_output(self, colour: str, ref_desc: str) -> None:
        """Build the output sentence and write it to the output text box."""
        corrected_up = self._compute_corrected_up_m()
        if corrected_up is not None:
            downwards_str = f"{corrected_up:.2f} m"
        elif (
            self._last_downwards is not None
            and self._last_downwards == self._last_downwards
        ):
            downwards_str = f"{self._last_downwards:.2f} m"
        else:
            downwards_str = "N/A"
        tx, ty = self._target_xy
        rx, ry = self._ref_xy
        direction = "left" if tx < rx else "right"
        lateral = self._compute_lateral_offset_m()
        if lateral is not None:
            lateral_str = f"{abs(lateral):.2f} m to the {direction}"
        else:
            lateral_str = f"to the {direction}"
        output = (
            f"The target is {colour}, and is located {downwards_str} off the ground and {lateral_str} "
            f"of the {ref_desc}."
        )
        self._output_text.config(state=tk.NORMAL)
        self._output_text.delete("1.0", tk.END)
        self._output_text.insert("1.0", output)
        self._output_text.config(state=tk.DISABLED)

    def _on_image_click(self, event: tk.Event) -> None:
        if self._click_mode is None:
            return
        if self._click_mode == SELECT_TARGET:
            self._target_xy = (event.x, event.y)
            self._redraw_image_with_crosshairs()
            self._click_mode = SELECT_REFERENCE
            if self._mode_var.get() == "full_manual":
                self._gen_prompt_label.config(
                    text="Select second point", fg=REFERENCE_CROSSHAIR_COLOUR
                )
            else:
                self._gen_prompt_label.config(
                    text="Select reference point", fg=REFERENCE_CROSSHAIR_COLOUR
                )
            return
        if self._click_mode == SELECT_REFERENCE:
            self._ref_xy = (event.x, event.y)
            self.image_label.unbind("<Button-1>")
            self._click_mode = None
            if self._mode_var.get() == "full_manual":
                self._draw_manual_measurements()
                self._gen_prompt_label.config(text="", fg="black")
            else:
                self._redraw_image_with_crosshairs()
                self._gen_prompt_label.config(
                    text="Fill the fields above and press Generate output to see the result.",
                    fg="black",
                )

    def _handle_capture_success(
        self,
        downwards_range: float,
        center_depth: float,
        pitch: float,
        roll: float,
        image: Image.Image,
        depth_map: np.ndarray,
    ) -> None:
        def fmt(v: float) -> str:
            if v != v:  # NaN check
                return "N/A"
            return f"{v:.2f} m"

        self.range_label.config(
            text=f"Downwards: {fmt(downwards_range)}    Depth(center): {fmt(center_depth)}"
        )
        self._last_downwards = downwards_range
        self._last_center_depth = center_depth
        self._last_pitch = pitch if pitch == pitch else None
        self._last_roll = roll if roll == roll else None
        if self._last_pitch is None:
            pitch_str = "N/A"
        else:
            pitch_deg = math.degrees(self._last_pitch)
            pitch_str = f"{pitch_deg:.1f}°"
        self.pitch_label.config(text=f"Pitch: {pitch_str}")
        if self._last_roll is None:
            roll_str = "N/A"
        else:
            roll_deg = math.degrees(self._last_roll)
            roll_str = f"{roll_deg:.1f}°"
        self.roll_label.config(text=f"Roll: {roll_str}")
        self.capture_status_label.config(text="")

        # Rotate image to remove roll before any geometric calculations.
        display_image = image.copy()
        rotated_depth_map = depth_map.copy()
        if self._last_roll is not None:
            # Negative roll de-rotates the image so the horizon appears level.
            roll_deg = math.degrees(self._last_roll)
            display_image = display_image.rotate(
                -roll_deg, expand=True, resample=Image.BICUBIC
            )
            rotated_depth_map = self._rotate_depth_map(rotated_depth_map, -roll_deg)

        # Draw red dot in center of (possibly rotated) image
        draw = ImageDraw.Draw(display_image)
        cx, cy = display_image.width // 2, display_image.height // 2
        r = 2  # radius in pixels
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill="red", outline="red")

        self._base_display_image = display_image.copy()
        self._base_depth_map = rotated_depth_map
        # Update image display
        photo = ImageTk.PhotoImage(image=display_image)
        self._current_photo = photo
        self.image_label.config(image=photo)

    def request_image(
        self,
    ) -> Tuple[float, float, float, float, Image.Image, np.ndarray]:
        """
        Connects to the transmitter, sends a capture command, and receives:
        - downward range (float32), center depth (float32), pitch (float32 rad), roll (float32 rad)
        - RGB JPEG length (uint64), depth PNG length (uint64)
        - JPEG image bytes, 16-bit PNG depth bytes
        Returns (downward_range_m, center_depth_m, pitch_rad, roll_rad, PIL.Image, depth_map_mm).
        """
        with socket.create_connection(
            (TRANSMITTER_HOST, TRANSMITTER_PORT), timeout=SOCKET_TIMEOUT
        ) as sock:
            sock.settimeout(SOCKET_TIMEOUT)

            # Send 1-byte capture command
            sock.sendall(b"C")

            header = self._recv_exact(sock, 32)
            if len(header) != 32:
                raise RuntimeError(
                    f"Incomplete header received (expected 32 bytes, got {len(header)})"
                )

            downwards_range, center_depth, pitch, roll, image_length, depth_length = (
                struct.unpack("!ffffQQ", header)
            )

            if image_length == 0:
                raise RuntimeError("Transmitter reported zero-length image")
            if depth_length == 0:
                raise RuntimeError("Transmitter reported zero-length depth map")

            jpeg_bytes = self._recv_exact(sock, image_length)
            if len(jpeg_bytes) != image_length:
                raise RuntimeError(
                    f"Incomplete image received (expected {image_length} bytes, got {len(jpeg_bytes)})"
                )
            depth_bytes = self._recv_exact(sock, depth_length)
            if len(depth_bytes) != depth_length:
                raise RuntimeError(
                    f"Incomplete depth map received (expected {depth_length} bytes, got {len(depth_bytes)})"
                )

        try:
            image = Image.open(io.BytesIO(jpeg_bytes))
            image.load()
        except Exception as exc:
            raise RuntimeError(f"Failed to decode JPEG image: {exc}") from exc

        depth_buffer = np.frombuffer(depth_bytes, dtype=np.uint8)
        depth_map = cv2.imdecode(depth_buffer, cv2.IMREAD_UNCHANGED)
        if depth_map is None:
            raise RuntimeError("Failed to decode depth PNG")
        if depth_map.ndim != 2:
            raise RuntimeError(f"Depth map has unexpected shape: {depth_map.shape}")

        return downwards_range, center_depth, pitch, roll, image, depth_map

    @staticmethod
    def _recv_exact(sock: socket.socket, num_bytes: int) -> bytes:
        """
        Receive exactly num_bytes from the socket, unless the connection closes or times out.
        """
        data = bytearray()
        while len(data) < num_bytes:
            chunk = sock.recv(num_bytes - len(data))
            if not chunk:
                break
            data.extend(chunk)
        return bytes(data)


def main() -> None:
    root = tk.Tk()
    app = ReceiverApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
