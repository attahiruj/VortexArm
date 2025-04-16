"""
SSC_32U Controller
This module provides a simple interface to control the SSC_32U servo controller.

Author: Attahiru Jibril
Date: 2025-04-14
"""

from dora import Node
import pyarrow as pa
from ssc32u import SSC_32U


def main():
    node = Node()
    
    # Define servo Pins for the SSC_32U controller
    base = 11
    shoulder = 10
    elbow = 7
    wrist = 6

    controller = SSC_32U(port="COM3", baud_rate=9600, timeout=1)
    controller.connect()

    if not controller.connected:
        print("Warning: SSC_32U not connected. Continuing without control.")
        # return

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "joints":# and controller.is_ready():
                joint_angles = event["value"]
                
                multiple_positions = [
                    (base, joint_angles[0].as_py()),        # shoulder
                    (shoulder, joint_angles[1].as_py()),    # Base
                    (elbow, joint_angles[2].as_py()),       # elbow
                ]

                controller.move_multiple_servos(multiple_positions, speed=100, angle=True)
                
                # Process the message and send commands to SSC_32U
                print(f"""
                      Received joint angles:
                      Base:{joint_angles[0]},
                      Shoulder:{joint_angles[1]},
                      Elbow:{joint_angles[2]}
                      """)



if __name__ == "__main__":
    main()
