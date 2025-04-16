"""
Main entry point for the inverse kinematics simulation.

Author: Attahiru Jibril
Date: 2025-04-14
"""

from dora import Node
import pyarrow as pa
from simulator import run_simulation
from inverse_kinematics import get_angles
from robot_config import robot_params


test_coordinates = [
    (0, 0, 0),
    (100, 100, 100),
    (150, 0, 150),
    (-100, -200, 200),
    (50, -50, 200),
]

def main():
    # run_simulation(robot_params)
    node = Node()

    for t in test_coordinates:
        x, y, z = t
        print(f"Testing coordinates: {t}")
        a_x, a_y, a_z = get_angles(x, y, z, robot_params)

        for event in node:
            if event["type"] == "INPUT":
                node.send_output(
                    "joints",
                    pa.array([a_x, a_y, a_z])
                )


if __name__ == "__main__":
    main()
