#!/usr/bin/env python3
"""
trajectory_replayer.py
======================
Replays a saved OpenArm bimanual trajectory YAML file on the real robot
using FollowJointTrajectory action clients — one per arm/gripper.

Usage:
  ros2 run <your_pkg> trajectory_replayer.py trajectories/handover_demo_v1.yaml

The four groups execute in the order stored in the YAML metadata:
  right_arm → right_gripper → left_arm → left_gripper
(each waits for the previous to succeed before proceeding)
"""

import sys
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ── Controller action server names ────────────────────────────────────────────
# Adjust these to match your robot's controller configuration.
ACTION_SERVERS = {
    "right_arm":     "/right_joint_trajectory_controller/follow_joint_trajectory",
    "right_gripper": "/right_gripper_controller/follow_joint_trajectory",
    "left_arm":      "/left_joint_trajectory_controller/follow_joint_trajectory",
    "left_gripper":  "/left_gripper_controller/follow_joint_trajectory",
}
# ──────────────────────────────────────────────────────────────────────────────


def secs_to_duration(t: float) -> Duration:
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Duration(sec=sec, nanosec=nanosec)


def build_trajectory(group_data: dict) -> JointTrajectory:
    traj = JointTrajectory()
    traj.joint_names = group_data["joint_names"]

    for pt in group_data["points"]:
        p = JointTrajectoryPoint()
        p.positions    = [float(v) for v in pt["positions"]]
        p.velocities   = [float(v) for v in pt["velocities"]]
        p.accelerations = [float(v) for v in pt["accelerations"]]
        p.time_from_start = secs_to_duration(pt["time_from_start"])
        traj.points.append(p)

    return traj


class TrajectoryReplayer(Node):
    def __init__(self, yaml_path: str):
        super().__init__("trajectory_replayer")

        with open(yaml_path, "r") as f:
            self._data = yaml.safe_load(f)

        meta = self._data.get("metadata", {})
        self.get_logger().info(
            f"Loaded trajectory: '{meta.get('name', '?')}' "
            f"recorded at {meta.get('recorded_at', '?')}"
        )

        self._clients: dict[str, ActionClient] = {}
        for group, server in ACTION_SERVERS.items():
            self._clients[group] = ActionClient(self, FollowJointTrajectory, server)

        self._execution_order: list[str] = self._data["metadata"].get(
            "execution_order",
            ["right_arm", "right_gripper", "left_arm", "left_gripper"],
        )

    def wait_for_servers(self, timeout_sec: float = 10.0):
        for group, client in self._clients.items():
            self.get_logger().info(f"Waiting for action server: {ACTION_SERVERS[group]}")
            if not client.wait_for_server(timeout_sec=timeout_sec):
                self.get_logger().error(
                    f"Action server not available: {ACTION_SERVERS[group]}"
                )
                return False
        return True

    def execute_group(self, group: str) -> bool:
        if group not in self._data:
            self.get_logger().warn(f"Group '{group}' not in YAML — skipping.")
            return True

        group_data = self._data[group]
        if not group_data.get("points"):
            self.get_logger().warn(f"Group '{group}' has no points — skipping.")
            return True

        traj = build_trajectory(group_data)
        traj.header.stamp = self.get_clock().now().to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info(
            f"Sending {group}: {len(traj.points)} points, "
            f"duration={traj.points[-1].time_from_start.sec:.2f}s"
        )

        client = self._clients[group]
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected for {group}!")
            return False

        self.get_logger().info(f"Goal accepted for {group}. Executing…")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"Execution failed for {group}: error_code={result.error_code} "
                f"'{result.error_string}'"
            )
            return False

        self.get_logger().info(f"✔ {group} done.")
        return True

    def run(self):
        order = self._data["metadata"]["execution_order"]
        for stage_name in order:
            success = self.execute_group(stage_name)
            if not success:
                self.get_logger().error(f"Halting at stage: '{stage_name}'")
                return


def main():
    if len(sys.argv) < 2:
        print("Usage: trajectory_replayer.py <path/to/trajectory.yaml>")
        sys.exit(1)

    yaml_path = sys.argv[1]

    rclpy.init(args=sys.argv)
    node = TrajectoryReplayer(yaml_path)

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
