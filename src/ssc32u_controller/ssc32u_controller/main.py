"""
SSC_32U Controller
This module provides a simple interface to control the SSC_32U servo controller.

Author: Attahiru Jibril
Date: 2025-04-14
"""

from dora import Node
import pyarrow as pa
import ssc32u_controller as ssc32u


def main():
    node = Node()

    controller = ssc32u.SSC_32U(port="COM3", baud_rate=9600, timeout=1)
    controller.connect()

    if not controller.connected:
        print("Warning: SSC_32U not connected. Continuing without control.")
        return

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "JOINTS":
                print(
                    f"""Node received:
                id: {event["id"]},
                value: {event["value"]},
                metadata: {event["metadata"]}"""
                )

            elif event["id"] == "my_input_id":
                # Warning: Make sure to add my_output_id and my_input_id within the dataflow.
                node.send_output(
                    output_id="my_output_id", data=pa.array([1, 2, 3]), metadata={}
                )
    # for event in node:
    #     if event["type"] == "INPUT":
    #         if event["id"] == "TICK":
    #             print(
    #                 f"""Node received:
    #             id: {event["id"]},
    #             value: {event["value"]},
    #             metadata: {event["metadata"]}"""
    #             )

    #         elif event["id"] == "my_input_id":
    #             # Warning: Make sure to add my_output_id and my_input_id within the dataflow.
    #             node.send_output(
    #                 output_id="my_output_id", data=pa.array([1, 2, 3]), metadata={}
    #             )


if __name__ == "__main__":
    main()
