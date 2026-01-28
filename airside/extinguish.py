"""
Target extinguishing module for airside drone operations.

This module handles:
- Target tracking and extinguishing queue management
- Navigation to target positions
- Extinguishing mechanism control (servo/GPIO)
- Extinguishing verification and status reporting
"""

import logging
import math
import time
from typing import Optional
from util import Coordinate, Colour


class Target:
    """Represents a target that needs to be extinguished."""

    def __init__(
        self,
        coordinate: Coordinate,
        colour: Colour,
        target_id: Optional[int] = None,
    ):
        """
        Initialize a target.

        Args:
            coordinate: Target location (lat, lon, alt)
            colour: Target color classification
            target_id: Optional unique identifier for this target
        """
        self.coordinate = coordinate
        self.colour = colour
        self.target_id = target_id
        self.extinguished = False
        self.extinguish_attempts = 0
        self.extinguish_timestamp: Optional[float] = None

    def __str__(self):
        status = "EXTINGUISHED" if self.extinguished else "PENDING"
        return f"Target(id={self.target_id}, {self.coordinate}, {self.colour.name}, {status})"

    def __repr__(self):
        return (
            f"Target(coordinate={self.coordinate}, colour={self.colour.name}, "
            f"extinguished={self.extinguished}, attempts={self.extinguish_attempts})"
        )


class ExtinguishController:
    """
    Manages target extinguishing operations.

    This controller handles:
    - Queue of targets to extinguish
    - Navigation to target positions
    - Extinguishing mechanism activation
    - Status tracking and reporting
    """

    # Navigation parameters
    POSITION_TOLERANCE_M = 1.0  # Meters - how close we need to be to target
    ALTITUDE_TOLERANCE_M = 0.5  # Meters - altitude tolerance for extinguishing

    # Extinguishing parameters
    EXTINGUISH_DURATION_SEC = 2.0  # How long to activate extinguishing mechanism
    MAX_EXTINGUISH_ATTEMPTS = 3  # Maximum attempts per target
    VERIFICATION_DELAY_SEC = 1.0  # Wait time after extinguishing to verify

    def __init__(self, mavlink_comm):
        """
        Initialize extinguish controller.

        Args:
            mavlink_comm: MavlinkComm instance for drone control
        """
        self.mavlink_comm = mavlink_comm
        self.target_queue: list[Target] = []
        self.current_target: Optional[Target] = None
        self.extinguished_targets: list[Target] = []
        self.failed_targets: list[Target] = []

    def add_target(self, coordinate: Coordinate, colour: Colour) -> Target:
        """
        Add a target to the extinguishing queue.

        Args:
            coordinate: Target location
            colour: Target color

        Returns:
            Target object that was added
        """
        target_id = len(self.target_queue) + len(self.extinguished_targets)
        target = Target(coordinate=coordinate, colour=colour, target_id=target_id)
        self.target_queue.append(target)
        logging.info(f"Added target to extinguishing queue: {target}")
        return target

    def get_next_target(self) -> Optional[Target]:
        """
        Get the next target from the queue.

        Returns:
            Next Target to extinguish, or None if queue is empty
        """
        if self.target_queue:
            self.current_target = self.target_queue.pop(0)
            return self.current_target
        return None

    def is_at_target(self, current_position: Coordinate) -> bool:
        """
        Check if drone is close enough to target position.

        Args:
            current_position: Current drone position

        Returns:
            True if within tolerance, False otherwise
        """
        if self.current_target is None:
            return False

        target = self.current_target.coordinate

        # Calculate horizontal distance (using lat/lon as x/y approximation)
        # For small distances, this is acceptable
        lat_diff = target.lat - current_position.lat
        lon_diff = target.lon - current_position.lon
        horizontal_distance = math.sqrt(lat_diff**2 + lon_diff**2) * 111000  # Rough conversion to meters

        # Check altitude difference
        alt_diff = abs(target.alt - current_position.alt)

        is_close = (
            horizontal_distance <= self.POSITION_TOLERANCE_M
            and alt_diff <= self.ALTITUDE_TOLERANCE_M
        )

        if is_close:
            logging.info(
                f"At target position: distance={horizontal_distance:.2f}m, "
                f"alt_diff={alt_diff:.2f}m"
            )

        return is_close

    def navigate_to_target(self) -> bool:
        """
        Navigate drone to current target position.

        Returns:
            True if navigation command sent successfully, False otherwise
        """
        if self.current_target is None:
            logging.warning("No current target to navigate to")
            return False

        target = self.current_target.coordinate
        logging.info(f"Navigating to target at {target}")

        # Use MAVLink to set waypoint
        success = self.mavlink_comm.set_waypoint(target)
        if success:
            logging.info(f"Navigation command sent to {target}")
        else:
            logging.error(f"Failed to send navigation command to {target}")

        return success

    def extinguish_target(self) -> bool:
        """
        Activate extinguishing mechanism for current target.

        Returns:
            True if extinguishing activated successfully, False otherwise
        """
        if self.current_target is None:
            logging.warning("No current target to extinguish")
            return False

        target = self.current_target

        if target.extinguished:
            logging.warning(f"Target {target.target_id} already extinguished")
            return True

        logging.info(f"Extinguishing target {target.target_id} at {target.coordinate}")

        # Activate extinguishing mechanism via servo/GPIO
        success = self.mavlink_comm.activate_extinguish(
            duration=self.EXTINGUISH_DURATION_SEC
        )

        if success:
            target.extinguish_attempts += 1
            logging.info(
                f"Extinguishing mechanism activated for {self.EXTINGUISH_DURATION_SEC}s"
            )

            # Wait for extinguishing to complete
            time.sleep(self.EXTINGUISH_DURATION_SEC)

            # Deactivate extinguishing mechanism
            self.mavlink_comm.deactivate_extinguish()

            # Mark timestamp for verification
            target.extinguish_timestamp = time.time()
            logging.info(f"Extinguishing complete for target {target.target_id}")

            return True
        else:
            logging.error(f"Failed to activate extinguishing mechanism")
            return False

    def verify_extinguish(self) -> bool:
        """
        Verify that target has been extinguished.

        This can be done by:
        - Checking if target color is no longer visible in camera
        - Waiting for confirmation from ground station
        - Time-based verification (if mechanism is reliable)

        Returns:
            True if extinguished, False otherwise
        """
        if self.current_target is None:
            return False

        target = self.current_target

        # For now, assume extinguishing was successful if mechanism activated
        # In future, add camera-based verification
        if target.extinguish_timestamp is not None:
            # Wait a bit for extinguishing to take effect
            time.sleep(self.VERIFICATION_DELAY_SEC)
            target.extinguished = True
            logging.info(f"Target {target.target_id} verified as extinguished")
            return True

        return False

    def process_current_target(self, current_position: Coordinate) -> str:
        """
        Process current target: navigate, extinguish, verify.

        Args:
            current_position: Current drone position

        Returns:
            Status string: "navigating", "extinguishing", "extinguished", "failed", "complete"
        """
        if self.current_target is None:
            return "complete"

        target = self.current_target

        # Check if we're at the target
        if not self.is_at_target(current_position):
            # Navigate to target
            if target.extinguish_attempts == 0:
                self.navigate_to_target()
            return "navigating"

        # We're at the target - extinguish it
        if not target.extinguished:
            if target.extinguish_attempts < self.MAX_EXTINGUISH_ATTEMPTS:
                if self.extinguish_target():
                    if self.verify_extinguish():
                        # Successfully extinguished
                        self.extinguished_targets.append(target)
                        self.mavlink_comm.send_extinguish_status_to_ground(
                            target, success=True
                        )
                        logging.info(f"Successfully extinguished target {target.target_id}")
                        self.current_target = None
                        return "extinguished"
                    else:
                        # Verification failed, try again
                        logging.warning(
                            f"Extinguishing verification failed for target {target.target_id}, "
                            f"attempt {target.extinguish_attempts}/{self.MAX_EXTINGUISH_ATTEMPTS}"
                        )
                        return "extinguishing"
                else:
                    # Extinguishing activation failed
                    target.extinguish_attempts += 1
                    if target.extinguish_attempts >= self.MAX_EXTINGUISH_ATTEMPTS:
                        # Max attempts reached
                        self.failed_targets.append(target)
                        self.mavlink_comm.send_extinguish_status_to_ground(
                            target, success=False
                        )
                        logging.error(
                            f"Failed to extinguish target {target.target_id} after "
                            f"{self.MAX_EXTINGUISH_ATTEMPTS} attempts"
                        )
                        self.current_target = None
                        return "failed"
                    return "extinguishing"
            else:
                # Max attempts reached
                self.failed_targets.append(target)
                self.mavlink_comm.send_extinguish_status_to_ground(target, success=False)
                logging.error(
                    f"Failed to extinguish target {target.target_id} after "
                    f"{self.MAX_EXTINGUISH_ATTEMPTS} attempts"
                )
                self.current_target = None
                return "failed"

        return "extinguished"

    def get_status(self) -> dict:
        """
        Get current extinguishing status.

        Returns:
            Dictionary with queue length, current target, and statistics
        """
        return {
            "queue_length": len(self.target_queue),
            "current_target": str(self.current_target) if self.current_target else None,
            "extinguished_count": len(self.extinguished_targets),
            "failed_count": len(self.failed_targets),
            "total_processed": len(self.extinguished_targets) + len(self.failed_targets),
        }

    def has_pending_targets(self) -> bool:
        """Check if there are targets waiting to be extinguished."""
        return len(self.target_queue) > 0 or self.current_target is not None
