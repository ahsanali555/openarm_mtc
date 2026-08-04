#!/usr/bin/env python3
"""
trajectory_recorder.py
=======================
Records joint trajectories from /joint_states during MTC execution on the
OpenArm bimanual robot and saves them to a single YAML file.

Usage:
  1. Launch MoveIt 2 + the MTC node.
  2. In RViz, pick the MTC solution you want.
  3. Run this node in a separate terminal.
  4. Type  start  and press Enter just before you click "Execute" in RViz.
  5. Type  stop   and press Enter once execution finishes.
  6. Enter a name (e.g. "handover_demo_v1") — saved to trajectories/<name>.yaml

Replay:
  Run trajectory_replayer.py with the saved YAML file.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml
import numpy as np
import threading
import os
import sys
import time
from copy import deepcopy


# ── Robot configuration ────────────────────────────────────────────────────────
# Adjust these joint name lists to match your URDF exactly.
RIGHT_ARM_JOINTS = [
    "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
    "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
]
RIGHT_GRIPPER_JOINTS = ["openarm_right_finger_joint1"]   # or whatever your gripper joint is named

LEFT_ARM_JOINTS = [
    "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
    "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
]
LEFT_GRIPPER_JOINTS = ["openarm_left_finger_joint1"]

# Velocity threshold below which a point is considered "at rest" (rad/s)
VELOCITY_ZERO_THRESHOLD = 0.005

# Minimum number of consecutive "moving" samples to keep a segment
MIN_MOVING_SAMPLES = 5

# Output directory (relative to where you run the script)
OUTPUT_DIR = "trajectories"
# ──────────────────────────────────────────────────────────────────────────────


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__("trajectory_recorder")

        self._lock = threading.Lock()
        self._recording = False
        self._buffer: list[JointState] = []
        self._start_time: float | None = None

        self._sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_cb,
            50,
        )

        os.makedirs(OUTPUT_DIR, exist_ok=True)
        self.get_logger().info("Trajectory recorder ready.")
        self.get_logger().info(
            "Commands (type in this terminal):\n"
            "  start  → begin recording\n"
            "  stop   → stop recording and save\n"
            "  quit   → exit without saving"
        )

    # ── ROS callback ──────────────────────────────────────────────────────────

    def _joint_state_cb(self, msg: JointState):
        with self._lock:
            if self._recording:
                if self._start_time is None:
                    self._start_time = self._ros_stamp(msg)
                self._buffer.append(deepcopy(msg))

    @staticmethod
    def _ros_stamp(msg: JointState) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    # ── Control interface ─────────────────────────────────────────────────────

    def start_recording(self):
        with self._lock:
            self._buffer.clear()
            self._start_time = None
            self._recording = True
        self.get_logger().info("● Recording started — execute the MTC task in RViz now.")

    def stop_recording(self) -> list[JointState]:
        with self._lock:
            self._recording = False
            data = list(self._buffer)
        self.get_logger().info(f"■ Recording stopped — {len(data)} samples captured.")
        return data

    # ── Processing ────────────────────────────────────────────────────────────

    def _extract_group(
        self,
        samples: list[JointState],
        joint_names: list[str],
        t0: float,
    ) -> dict:
        """
        Pull one group's joints out of the raw JointState stream.
        Returns a dict with keys: joint_names, points (each point has
        time_from_start, positions, velocities, accelerations).
        """
        points = []
        for msg in samples:
            # Build index map for this message (order may vary between messages)
            name_to_idx = {n: i for i, n in enumerate(msg.name)}
            if not all(j in name_to_idx for j in joint_names):
                continue  # message doesn't contain all joints — skip

            t = self._ros_stamp(msg) - t0
            pos = [msg.position[name_to_idx[j]] for j in joint_names]
            vel = (
                [msg.velocity[name_to_idx[j]] for j in joint_names]
                if len(msg.velocity) == len(msg.name)
                else [0.0] * len(joint_names)
            )
            acc = (
                [msg.effort[name_to_idx[j]] for j in joint_names]
                if len(msg.effort) == len(msg.name)
                else [0.0] * len(joint_names)
            )
            points.append({"t": t, "pos": pos, "vel": vel, "acc": acc})

        return {"joint_names": joint_names, "raw_points": points}

    @staticmethod
    def _trim_endpoints(points: list[dict]) -> list[dict]:
        """
        Remove leading and trailing points where the robot is stationary
        (all joint velocities below threshold).

        Works in two passes:
          1. Find first index where any joint moves (leading trim)
          2. Find last index where any joint moves (trailing trim)
        Then clamp, but keep at least MIN_MOVING_SAMPLES points.
        """
        if not points:
            return points

        def is_moving(pt: dict) -> bool:
            return any(abs(v) > VELOCITY_ZERO_THRESHOLD for v in pt["vel"])

        # Leading trim: find first run of MIN_MOVING_SAMPLES consecutive moving points
        first_moving = None
        consecutive = 0
        for i, pt in enumerate(points):
            if is_moving(pt):
                if first_moving is None:
                    first_moving = i
                consecutive += 1
                if consecutive >= MIN_MOVING_SAMPLES:
                    break
            else:
                first_moving = None
                consecutive = 0

        if first_moving is None:
            # Entire segment is stationary — return as-is with a warning
            return points

        # Trailing trim: same logic from the end
        last_moving = None
        consecutive = 0
        for i in range(len(points) - 1, -1, -1):
            if is_moving(points[i]):
                if last_moving is None:
                    last_moving = i
                consecutive += 1
                if consecutive >= MIN_MOVING_SAMPLES:
                    break
            else:
                last_moving = None
                consecutive = 0

        if last_moving is None:
            last_moving = len(points) - 1

        trimmed = points[first_moving : last_moving + 1]

        # Re-zero timestamps so each group starts at t=0
        t_offset = trimmed[0]["t"]
        for pt in trimmed:
            pt["t"] -= t_offset

        return trimmed

    @staticmethod
    def _to_yaml_points(points: list[dict]) -> list[dict]:
        """Convert internal point dicts to clean YAML-serialisable dicts."""
        return [
            {
                "time_from_start": round(pt["t"], 6),
                "positions": [round(v, 8) for v in pt["pos"]],
                "velocities": [round(v, 8) for v in pt["vel"]],
                "accelerations": [round(v, 8) for v in pt["acc"]],
            }
            for pt in points
        ]

    def process_and_save(self, raw: list[JointState], name: str):
        if not raw:
            self.get_logger().error("No data recorded — nothing to save.")
            return

        t0 = self._ros_stamp(raw[0])

        # ── Extract each group ──────────────────────────────────────────────
        right_arm_data   = self._extract_group(raw, RIGHT_ARM_JOINTS,     t0)
        right_gripper_data = self._extract_group(raw, RIGHT_GRIPPER_JOINTS, t0)
        left_arm_data    = self._extract_group(raw, LEFT_ARM_JOINTS,      t0)
        left_gripper_data  = self._extract_group(raw, LEFT_GRIPPER_JOINTS,  t0)

        # ── Trim stationary endpoints ───────────────────────────────────────
        right_arm_pts   = self._trim_endpoints(right_arm_data["raw_points"])
        right_grip_pts  = self._trim_endpoints(right_gripper_data["raw_points"])
        left_arm_pts    = self._trim_endpoints(left_arm_data["raw_points"])
        left_grip_pts   = self._trim_endpoints(left_gripper_data["raw_points"])

        # ── Build YAML structure ────────────────────────────────────────────
        doc = {
            "metadata": {
                "name": name,
                "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "description": (
                    "OpenArm bimanual MTC handover trajectory. "
                    "right_arm executes first, then left_arm."
                ),
                "execution_order": ["right_arm", "right_gripper", "left_arm", "left_gripper"],
                "velocity_zero_threshold": VELOCITY_ZERO_THRESHOLD,
                "raw_samples_captured": len(raw),
            },
            "right_arm": {
                "joint_names": RIGHT_ARM_JOINTS,
                "num_points": len(right_arm_pts),
                "points": self._to_yaml_points(right_arm_pts),
            },
            "right_gripper": {
                "joint_names": RIGHT_GRIPPER_JOINTS,
                "num_points": len(right_grip_pts),
                "points": self._to_yaml_points(right_grip_pts),
            },
            "left_arm": {
                "joint_names": LEFT_ARM_JOINTS,
                "num_points": len(left_arm_pts),
                "points": self._to_yaml_points(left_arm_pts),
            },
            "left_gripper": {
                "joint_names": LEFT_GRIPPER_JOINTS,
                "num_points": len(left_grip_pts),
                "points": self._to_yaml_points(left_grip_pts),
            },
        }

        path = os.path.join(OUTPUT_DIR, f"{name}.yaml")
        with open(path, "w") as f:
            yaml.dump(doc, f, default_flow_style=False, sort_keys=False)

        self.get_logger().info(f"✔ Saved → {os.path.abspath(path)}")
        self.get_logger().info(
            f"  right_arm:    {len(right_arm_pts)} points  "
            f"(trimmed from {len(right_arm_data['raw_points'])})"
        )
        self.get_logger().info(
            f"  right_gripper:{len(right_grip_pts)} points  "
            f"(trimmed from {len(right_gripper_data['raw_points'])})"
        )
        self.get_logger().info(
            f"  left_arm:     {len(left_arm_pts)} points  "
            f"(trimmed from {len(left_arm_data['raw_points'])})"
        )
        self.get_logger().info(
            f"  left_gripper: {len(left_grip_pts)} points  "
            f"(trimmed from {len(left_gripper_data['raw_points'])})"
        )


# ── CLI loop (runs in main thread; ROS spins in background) ───────────────────

def cli_loop(recorder: TrajectoryRecorder):
    raw_data = None
    print("\nReady. Commands: start | stop | quit\n")
    while rclpy.ok():
        try:
            cmd = input(">> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd == "start":
            recorder.start_recording()

        elif cmd == "stop":
            raw_data = recorder.stop_recording()
            if not raw_data:
                print("No data was captured. Did you type 'start' first?")
                continue
            name = input("Enter trajectory name (no spaces): ").strip()
            if not name:
                name = f"trajectory_{int(time.time())}"
            recorder.process_and_save(raw_data, name)
            print("Done. Type 'start' to record another, or 'quit' to exit.")

        elif cmd == "quit":
            print("Exiting.")
            break

        else:
            print("Unknown command. Use: start | stop | quit")


def main():
    rclpy.init(args=sys.argv)
    recorder = TrajectoryRecorder()

    # Spin ROS in a background thread so the CLI can block on input()
    spin_thread = threading.Thread(target=rclpy.spin, args=(recorder,), daemon=True)
    spin_thread.start()

    try:
        cli_loop(recorder)
    finally:
        recorder.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
