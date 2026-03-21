#!/usr/bin/env python3
"""
check_nav2_slam_ready.py

Checks (and prints) whether common SLAM + Nav2 components look "ready" on ROS 2:
- Expected nodes exist
- Lifecycle nodes are ACTIVE (via /<node>/get_state) when available
- Key Nav2 action servers are available
- (Optional) required topics have published at least one message
- (Optional) required TF transforms are available

Usage examples:
  ros2 run <your_pkg> check_nav2_slam_ready.py
  python3 check_nav2_slam_ready.py --timeout 10
  python3 check_nav2_slam_ready.py --no-topics --no-tf
  python3 check_nav2_slam_ready.py --expect slam_toolbox planner_server controller_server

Exit code:
  0 if all checks pass, else 1
"""
import argparse
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from lifecycle_msgs.srv import GetState

# Nav2 actions
from nav2_msgs.action import NavigateToPose, FollowWaypoints

# Topic message types (used only if you enable topic checks)
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage

# TF check
from tf2_ros import Buffer, TransformListener


DEFAULT_EXPECT_NODES = [
    # SLAM
    "slam_toolbox",

    # Nav2 core servers (typical)
    "bt_navigator",
    "controller_server",
    "planner_server",
    "recoveries_server",
    "behavior_server",       # present in many Nav2 configs
    "smoother_server",       # optional depending on config
    "waypoint_follower",     # optional depending on config

    # Localization/map (you might have slam_toolbox instead of these)
    "map_server",            # optional in SLAM configs
    "amcl",                  # optional in SLAM configs

    # Lifecycle managers (names vary by bringup)
    "lifecycle_manager_navigation",
    "lifecycle_manager_localization",
]

DEFAULT_ACTIONS = [
    ("navigate_to_pose", NavigateToPose),
    ("follow_waypoints", FollowWaypoints),
]

DEFAULT_TOPIC_CHECKS = [
    ("scan", LaserScan),
    ("map", OccupancyGrid),
    ("tf", TFMessage),
]


DEFAULT_TF_CHECKS = [
    ("map", "odom"),
    ("odom", "base_link"),
]


LIFECYCLE_ACTIVE_ID = 3  # lifecycle_msgs/State.PRIMARY_STATE_ACTIVE


class ReadyChecker(Node):
    def __init__(self, namespace: str) -> None:
        super().__init__("slam_nav2_ready_checker", namespace=namespace)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def list_all_node_names(self, discovery_timeout_s: float = 0.0) -> List[str]:
        """
        Return both base and fully-qualified node names.
        Optional discovery timeout helps DDS graph populate before querying.
        """
        end = time.time() + max(0.0, discovery_timeout_s)
        names_and_ns = self.get_node_names_and_namespaces()
        while time.time() < end and rclpy.ok() and len(names_and_ns) <= 1:
            rclpy.spin_once(self, timeout_sec=0.05)
            names_and_ns = self.get_node_names_and_namespaces()

        names: List[str] = []
        for (name, ns) in names_and_ns:
            # ns usually '/' or '/robot1', keep just base node name for matching
            # but also keep fully qualified for debug.
            names.append(name)
            names.append(f"{ns.rstrip('/')}/{name}" if ns != "/" else f"/{name}")
        return sorted(set(names))

    def node_exists(self, node_name: str) -> bool:
        return node_name in self.list_all_node_names()

    def lifecycle_state(self, node_name: str, timeout_s: float) -> Optional[Tuple[int, str]]:
        """
        Try calling /<node_name>/get_state.
        Returns (state_id, state_label) if service exists and responds, else None.
        """
        srv_name = f"{node_name}/get_state" if node_name.startswith("/") else f"/{node_name}/get_state"
        client = self.create_client(GetState, srv_name)

        end = time.time() + timeout_s
        while time.time() < end and not client.wait_for_service(timeout_sec=0.2):
            rclpy.spin_once(self, timeout_sec=0.05)

        if not client.service_is_ready():
            self.destroy_client(client)
            return None

        req = GetState.Request()
        future = client.call_async(req)

        end_call = time.time() + timeout_s
        while time.time() < end_call and rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)

        if not future.done():
            self.destroy_client(client)
            return None

        try:
            resp = future.result()
        except Exception:
            self.destroy_client(client)
            return None

        self.destroy_client(client)
        return (resp.current_state.id, resp.current_state.label)

    def action_server_ready(self, action_name: str, action_type, timeout_s: float) -> bool:
        client = ActionClient(self, action_type, action_name)
        end = time.time() + timeout_s
        while time.time() < end and rclpy.ok() and not client.wait_for_server(timeout_sec=0.2):
            rclpy.spin_once(self, timeout_sec=0.05)
        ok = client.server_is_ready()
        client.destroy()
        return ok

    def topic_has_message(self, topic: str, msg_type, timeout_s: float) -> bool:
        got = {"ok": False}

        def cb(_msg):
            got["ok"] = True

        sub = self.create_subscription(msg_type, topic, cb, 10)

        end = time.time() + timeout_s
        while time.time() < end and rclpy.ok() and not got["ok"]:
            rclpy.spin_once(self, timeout_sec=0.05)

        self.destroy_subscription(sub)
        return got["ok"]

    def tf_available(self, target: str, source: str, timeout_s: float) -> bool:
        end = time.time() + timeout_s
        while time.time() < end and rclpy.ok():
            try:
                # latest available
                self._tf_buffer.lookup_transform(target, source, rclpy.time.Time())
                return True
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.05)
        return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=8.0, help="Per-check timeout in seconds.")
    parser.add_argument(
        "--namespace",
        type=str,
        default="/robot_0",
        help="Namespace prefix for nodes/actions/topics (e.g., /robot1).",
    )
    parser.add_argument(
        "--expect",
        nargs="*",
        default=None,
        help="Override expected node names (space-separated).",
    )
    parser.add_argument(
        "--discovery-timeout",
        type=float,
        default=5.0,
        help="Time to wait for ROS graph discovery before listing nodes.",
    )
    parser.add_argument(
        "--actions",
        nargs="*",
        default=None,
        help="Override action names to check (space-separated). Types remain NavigateToPose+FollowWaypoints for defaults.",
    )
    parser.add_argument("--no-lifecycle", action="store_true", help="Skip lifecycle get_state checks.")
    parser.add_argument("--no-actions", action="store_true", help="Skip action server checks.")
    parser.add_argument("--no-topics", action="store_true", help="Skip topic message checks.")
    parser.add_argument("--no-tf", action="store_true", help="Skip TF checks.")
    parser.add_argument("--strict-nodes", action="store_true", help="Fail if ANY expected node is missing.")
    args = parser.parse_args()

    def normalize_namespace(ns: str) -> str:
        ns = ns.strip()
        if not ns or ns == "/":
            return ""
        return ns if ns.startswith("/") else f"/{ns}"

    def qualify(name: str, ns: str) -> str:
        # Treat defaults as relative names. If user passes an absolute name, keep it absolute.
        if not ns:
            return name if name.startswith("/") else f"/{name}"
        if name.startswith("/"):
            return name  # absolute path provided explicitly
        return f"{ns.rstrip('/')}/{name}"


    rclpy.init()
    namespace = normalize_namespace(args.namespace)
    node = ReadyChecker(namespace)

    timeout = float(args.timeout)
    raw_expected_nodes = args.expect if args.expect is not None else list(DEFAULT_EXPECT_NODES)
    expected_nodes = [qualify(n, namespace) for n in raw_expected_nodes]

    # If user overrides actions by name, we can only reliably type-check defaults;
    # so custom list just checks server existence for NavigateToPose (best-effort).
    actions_to_check: List[Tuple[str, object]] = []
    if not args.no_actions:
        if args.actions is None:
            actions_to_check = [(qualify(name, namespace), action_type) for name, action_type in DEFAULT_ACTIONS]
        else:
            # Best-effort: assume NavigateToPose for custom action names unless they match follow_waypoints
            for name in args.actions:
                if "follow_waypoints" in name:
                    actions_to_check.append((qualify(name, namespace), FollowWaypoints))
                else:
                    actions_to_check.append((qualify(name, namespace), NavigateToPose))

    failures: List[str] = []
    warnings: List[str] = []

    print("=== ROS 2 SLAM + Nav2 readiness check ===")
    all_nodes = set(node.list_all_node_names(discovery_timeout_s=float(args.discovery_timeout)))
    print(f"Discovered {len(all_nodes)} nodes on the graph.")


    # Node presence + lifecycle state
    print("\n-- Nodes / Lifecycle --")
    for n in expected_nodes:
        exists = (n in all_nodes) or (n.lstrip("/") in all_nodes)
        if not exists:
            msg = f"[MISSING] node '{n}' not found"
            if args.strict_nodes:
                failures.append(msg)
            else:
                warnings.append(msg)
            print(msg)
            continue

        print(f"[OK] node '{n}' found")

        if not args.no_lifecycle:
            st = node.lifecycle_state(n, timeout_s=timeout)
            if st is None:
                # Not necessarily a lifecycle node; treat as warning.
                w = f"[WARN] no lifecycle state service for '{n}' (/{n}/get_state not available or no response)"
                warnings.append(w)
                print(w)
            else:
                state_id, state_label = st
                if state_id == LIFECYCLE_ACTIVE_ID:
                    print(f"[OK] lifecycle '{n}' is ACTIVE ({state_label})")
                else:
                    failures.append(f"[NOT READY] lifecycle '{n}' is {state_label} (id={state_id}), expected ACTIVE")
                    print(f"[NOT READY] lifecycle '{n}' is {state_label} (id={state_id}), expected ACTIVE")

    # Action servers
    if not args.no_actions:
        print("\n-- Nav2 action servers --")
        for action_name, action_type in actions_to_check:
            ok = node.action_server_ready(action_name, action_type, timeout_s=timeout)
            if ok:
                print(f"[OK] action server '{action_name}' is available")
            else:
                failures.append(f"[MISSING] action server '{action_name}' not available")
                print(f"[MISSING] action server '{action_name}' not available")

    # Topic checks (message observed)
    if not args.no_topics:
        print("\n-- Topics (message observed) --")
        for topic, msg_type in DEFAULT_TOPIC_CHECKS:
            ok = node.topic_has_message(qualify(topic, namespace), msg_type, timeout_s=timeout)
            if ok:
                print(f"[OK] topic '{qualify(topic, namespace)}' has messages")
            else:
                failures.append(f"[NO DATA] topic '{qualify(topic, namespace)}' did not publish within {timeout:.1f}s")
                print(f"[NO DATA] topic '{qualify(topic, namespace)}' did not publish within {timeout:.1f}s")

    # TF checks
    if not args.no_tf:
        print("\n-- TF transforms --")
        for target, source in DEFAULT_TF_CHECKS:
            ok = node.tf_available(target, source, timeout_s=timeout)
            if ok:
                print(f"[OK] TF '{source}' -> '{target}' available")
            else:
                failures.append(f"[NO TF] TF '{source}' -> '{target}' not available within {timeout:.1f}s")
                print(f"[NO TF] TF '{source}' -> '{target}' not available within {timeout:.1f}s")

    print("\n=== Summary ===")
    if warnings:
        print(f"Warnings: {len(warnings)}")
        for w in warnings:
            print(f"  - {w}")
    else:
        print("Warnings: 0")

    if failures:
        print(f"Failures: {len(failures)}")
        for f in failures:
            print(f"  - {f}")
        print("\nResult: NOT READY")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print("Failures: 0")
    print("\nResult: READY")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
