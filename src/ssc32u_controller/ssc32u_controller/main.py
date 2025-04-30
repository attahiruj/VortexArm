"""
SSC_32U Controller
This module provides a simple interface to control the SSC_32U servo controller.

Author: Attahiru Jibril (Modified)
Date: 2025-04-20
"""

from dora import Node, DoraStatus
import pyarrow as pa
import sys
from ssc32u import SSC_32U
import time
from robot_config import servo_config


def main():
    """Main function to run the SSC_32U controller node"""
    node = Node()

    # Get servo pins from configuration
    base_pin = servo_config["base"]["pin"]
    shoulder_pin = servo_config["shoulder"]["pin"]
    elbow_pin = servo_config["elbow"]["pin"]
    wrist_pin = servo_config["wrist"]["pin"]

    # print("Initializing SSC_32U controller...")
    controller = SSC_32U(port="COM3", baud_rate=9600, timeout=1)

    connected = False
    if not controller.connected:
        connected = controller.connect()

    if not connected:
        # print("Warning: SSC_32U not connected. Will continue in simulation mode.", file=sys.stderr)
        pass

    # Send initial message confirming readiness
    node.send_output("speech", pa.array(["Robot arm controller initialized"]), {})

    # Process events in a loop
    for event in node:
        try:
            if event["type"] == "INPUT":
                if event["id"] == "joints":
                    joint_angles = event["value"]

                    # Check if we have the right number of angles
                    if len(joint_angles) < 3:
                       # print(f"Error: Expected 3 joint angles, got {len(joint_angles)}", file=sys.stderr)
                        node.send_output("speech", pa.array(["Invalid joint data received"]), {})
                        continue

                    # Get angle values, protect against type errors
                    try:
                        base_angle = float(joint_angles[0].as_py())
                        shoulder_angle = float(joint_angles[1].as_py())
                        elbow_angle = float(joint_angles[2].as_py())

                        # Check if angles are within servo limits
                        base_min = servo_config["base"]["min_angle"]
                        base_max = servo_config["base"]["max_angle"]
                        if base_angle < base_min or base_angle > base_max:
                           # print(f"Warning: Base angle {base_angle}° is outside range [{base_min}, {base_max}]")
                            base_angle = max(min(base_angle, base_max), base_min)
                            
                        shoulder_min = servo_config["shoulder"]["min_angle"]
                        shoulder_max = servo_config["shoulder"]["max_angle"]
                        if shoulder_angle < shoulder_min or shoulder_angle > shoulder_max:
                           # print(f"Warning: Shoulder angle {shoulder_angle}° is outside range [{shoulder_min}, {shoulder_max}]")
                            shoulder_angle = max(min(shoulder_angle, shoulder_max), shoulder_min)
                            
                        elbow_min = servo_config["elbow"]["min_angle"]
                        elbow_max = servo_config["elbow"]["max_angle"]
                        if elbow_angle < elbow_min or elbow_angle > elbow_max:
                           # print(f"Warning: Elbow angle {elbow_angle}° is outside range [{elbow_min}, {elbow_max}]")
                            elbow_angle = max(min(elbow_angle, elbow_max), elbow_min)

                        multiple_positions = [
                            (base_pin, base_angle),            # Base
                            (shoulder_pin, shoulder_angle),    # Shoulder
                            (elbow_pin, elbow_angle),          # Elbow
                        ]

                        # Print the joint angles for debugging
                        # print(f"""
                        #     Received joint angles:
                        #     Base: {base_angle}°,
                        #     Shoulder: {shoulder_angle}°,
                        #     Elbow: {elbow_angle}°
                        #     """)

                        # Send command to move servos if connected
                        if controller.connected:
                            controller.move_multiple_servos(multiple_positions, speed=100, angle=True)
                            node.send_output("speech", pa.array([f"Moving to position: B={base_angle:.1f}°, S={shoulder_angle:.1f}°, E={elbow_angle:.1f}°"]), {})
                        else:
                            # print("Simulating servo movement (not connected)", file=sys.stderr)
                            pass

                    except Exception as e:
                        # print(f"Error processing joint angles: {e}", file=sys.stderr)
                        node.send_output("speech", pa.array(["Error processing joint data"]), {})

                elif event["id"] == "tick":
                    # Just process the tick event to keep the node alive
                    pass

            elif event["type"] == "STOP":
                # Clean up on stop
                if controller.connected:
                    controller.disconnect()
                break

        except Exception as e:
            # print(f"Error in event loop: {e}", file=sys.stderr)
            # Continue running despite errors
            continue

    # Final cleanup
    if controller.connected:
        controller.disconnect()

    return 0


if __name__ == "__main__":
    main()