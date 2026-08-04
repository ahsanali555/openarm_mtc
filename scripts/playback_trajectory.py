#!/usr/bin/env python3
"""
playback_trajectory.py
Replay a saved MTC stage YAML on the real robot via FollowJointTrajectory.

Usage:
  ros2 run <pkg> playback_trajectory.py <path/to/stage.yaml> <controller_name>

Example:
  ros2 run openarm_mtc_pick_place playback_trajectory.py \
    /tmp/openarm_trajectories/openarm_traj_stage_5.yaml \
    right_joint_trajectory_controller
"""

import sys
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def to_float_list(raw):
    """Return a list of Python floats, or [] if raw is None / empty."""
    if not raw:
        return []
    return [float(v) for v in raw]


class TrajectoryPlayer(Node):
    def __init__(self, yaml_path: str, controller: str):
        super().__init__('trajectory_player')
        self._yaml_path = yaml_path
        self._controller = controller
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            f'/{controller}/follow_joint_trajectory'
        )

    def run(self):
        # ── load YAML ────────────────────────────────────────────────────
        with open(self._yaml_path) as f:
            data = yaml.safe_load(f)

        joint_names = data.get('joint_names', [])
        raw_points  = data.get('points', [])

        if not joint_names or not raw_points:
            self.get_logger().error(
                f'YAML has no joint_names or no points — skipping: {self._yaml_path}')
            return

        self.get_logger().info(
            f'Loaded {len(raw_points)} point(s) for joints: {joint_names}')

        # ── build JointTrajectory message ─────────────────────────────────
        traj = JointTrajectory()
        traj.joint_names = joint_names

        for raw in raw_points:
            pt = JointTrajectoryPoint()

            # positions are mandatory
            positions = to_float_list(raw.get('positions'))
            if not positions:
                self.get_logger().warn('Point with no positions — skipping point')
                continue
            pt.positions = positions

            # velocities / accelerations: only set when non-empty so the
            # controller does not reject a mismatched-length sequence
            velocities     = to_float_list(raw.get('velocities'))
            accelerations  = to_float_list(raw.get('accelerations'))
            if velocities:
                pt.velocities = velocities
            if accelerations:
                pt.accelerations = accelerations

            # timing
            t = float(raw.get('time_from_start_sec', 0.0))
            pt.time_from_start = Duration(
                sec=int(t),
                nanosec=int(round((t - int(t)) * 1e9))
            )

            traj.points.append(pt)

        if not traj.points:
            self.get_logger().error('No valid points after parsing — aborting')
            return

        # ── wait for action server ────────────────────────────────────────
        self.get_logger().info(
            f'Waiting for /{self._controller}/follow_joint_trajectory ...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'Action server not available after 10 s. '
                'Is the controller active? Run: ros2 control list_controllers')
            return

        # ── send goal ────────────────────────────────────────────────────
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self.get_logger().info(
            f'Sending {len(traj.points)} point(s) to {self._controller} ...')
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by controller')
            return

        self.get_logger().info('Goal accepted — executing ...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Trajectory execution SUCCESSFUL')
        else:
            self.get_logger().error(
                f'Trajectory execution FAILED — error_code={result.error_code} '
                f'error_string="{result.error_string}"')


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    yaml_path  = sys.argv[1]
    controller = sys.argv[2]

    rclpy.init()
    node = TrajectoryPlayer(yaml_path, controller)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
