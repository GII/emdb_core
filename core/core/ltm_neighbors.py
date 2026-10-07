"""Write a FileNeighborsFull-compatible file from an LTM service."""

import argparse
from pathlib import Path

import rclpy
import yaml
from core_interfaces.srv import GetNodeFromLTM
from rclpy.node import Node


HEADER = "NodeType\tNodeName\tNeighborName\tNeighborType\n"


def format_neighbors(state):
    """Return the tab-separated neighbor file contents for an LTM state."""
    lines = [HEADER]
    for node_type, node_dict in state.items():
        if not isinstance(node_dict, dict):
            continue
        for node_name, node_data in node_dict.items():
            neighbors = (
                node_data.get("neighbors", [])
                if isinstance(node_data, dict)
                else []
            )
            if neighbors:
                for neighbor in neighbors:
                    lines.append(
                        f"{node_type}\t{node_name}\t"
                        f"{neighbor['name']}\t{neighbor['node_type']}\n"
                    )
            else:
                lines.append(f"{node_type}\t{node_name}\t\t\n")
    return "".join(lines)


def read_ltm_state(service_name):
    """Read the complete state from an LTM get_node service."""
    node = Node("ltm_neighbors")
    client = node.create_client(GetNodeFromLTM, service_name)
    try:
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(
                f"Service {service_name} not available, waiting again..."
            )

        request = GetNodeFromLTM.Request()
        request.name = ""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        if response is None:
            raise RuntimeError(f"Service call to {service_name} failed")
        state = yaml.safe_load(response.data)
        if not isinstance(state, dict):
            raise ValueError(
                f"Service {service_name} returned a non-mapping LTM state"
            )
        return state
    finally:
        node.destroy_node()


def main(argv=None):
    """Read an LTM and write its FileNeighborsFull-compatible neighbor file."""
    parser = argparse.ArgumentParser(
        description=(
            "Read an LTM state and write a FileNeighborsFull-compatible file."
        )
    )
    parser.add_argument(
        "service_name",
        help="LTM get_node service name, for example /ltm_0/get_node",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        required=True,
        help="Output file to write",
    )
    args = parser.parse_args(argv)

    rclpy.init()
    try:
        state = read_ltm_state(args.service_name)
        args.output.write_text(format_neighbors(state), encoding="utf-8")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
