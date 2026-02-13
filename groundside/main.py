"""
Groundside main control loop for receiving and displaying drone target data.

This module listens for MAVLink STATUSTEXT messages containing:
- Building geometry data
- Target locations with color information
- Generates human-readable descriptions of target positions
"""

import logging
from groundside.building import Building
from groundside.mavlink_comm import MavlinkReceiver


def main() -> None:
    """Main loop for groundside station."""
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )
    logging.info("Starting groundside station...")

    building = Building()
    receiver = MavlinkReceiver()

    logging.info("Listening for drone messages...")

    try:
        while True:
            msg = receiver.process_messages()

            if msg is None:
                continue

            msg_type = msg["type"]
            data = msg["data"]

            if msg_type == "building_corner":
                building.add_corner(data)
                logging.info(f"Stored building corner {len(building.corners)}: {data}")

                if building.is_complete():
                    logging.info(f"Building fully mapped: {building}")

            elif msg_type == "target":
                coordinate = data["coordinate"]
                colour = data["colour"]

                description = building.generate_target_description(coordinate, colour)
                print(f"\n{'='*80}")
                print(f"TARGET DETECTED:")
                print(description)
                print(f"{'='*80}\n")

            elif msg_type == "ack":
                logging.info(f"Drone acknowledgement: {data}")

            elif msg_type == "pickled":
                logging.info(f"Received pickled object: {data}")

    except KeyboardInterrupt:
        logging.info("Shutting down groundside station...")


if __name__ == "__main__":
    main()
