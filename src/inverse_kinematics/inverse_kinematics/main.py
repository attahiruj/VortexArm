"""
Main entry point for the inverse kinematics simulation.

Author: Attahiru Jibril
Date: 2025-04-14
"""

from simulator import run_simulation
from robot_config import robot_params


def main():
    run_simulation(robot_params)


if __name__ == "__main__":
    main()
