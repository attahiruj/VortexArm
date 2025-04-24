"""
Main entry point for the inverse kinematics simulation.

Author: Attahiru Jibril (Modified)
Date: 2025-04-20
"""

from dora import Node, DoraStatus
import time
from robot_config import robot_params
from simulator import Operator as SimulatorOperator


def main():
    """Main function to run the Dora node system"""
    # Initialize the Dora node
    node = Node()

    # Create the simulator operator
    simulator = SimulatorOperator()

    # Force immediate initialization
    print("Starting simulator...")
    simulator.run_simulation(
        robot_params,
        lambda output_id,
        data,
        metadata: node.send_output(output_id, data, metadata)
    )

    print("Simulator started, entering event loop")

    # Process Dora events
    for event in node:
        # Process the event
        status = simulator.on_event(
            event,
            lambda output_id, data,
            metadata: node.send_output(output_id, data, metadata)
        )

        # Small sleep to prevent CPU hogging
        time.sleep(0.01)

        # Stop if requested
        if status == DoraStatus.STOP:
            break

    print("Exiting simulator")


if __name__ == "__main__":
    main()
