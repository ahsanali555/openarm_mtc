# OpenArm MTC Pick-Place & Handover

ROS 2 Humble | MoveIt 2 | MoveIt Task Constructor

Motion planning packages for the **OpenArm v10 bimanual robot** covering single-arm pick-and-place, bimanual sequential pick-and-place, and arm-to-arm handover — with trajectory saving and real-hardware playback.

![openarm_mtc](https://github.com/ahsanali555/openarm_mtc/blob/main/assets/openarm_mtc.png)

---

## Repository Structure

```
openarm_mtc_pick_place/
├── src/
│   ├── mtc_node.cpp                  # Single-arm right pick-and-place
│   ├── mtc_node_air_pickplace.cpp    # Gripper/motion test (no collision objects)
│   ├── mtc_handover_node.cpp         # Right picks → left picks same object
│   └── mtc_parallel_node.cpp         # Bimanual sequential pick-and-place
├── scripts/
│   └── playback_trajectory.py        # Replay saved YAML trajectories on hardware
├── launch/
│   ├── mtc_bringup.launch.py         # Full stack: robot_state_pub + controllers + MoveIt
│   ├── pick_place_demo.launch.py     # Single-arm pick-and-place
│   ├── air_pickplace_demo.launch.py  # Air pick-and-place (no objects)
│   ├── handover_demo.launch.py       # Bimanual handover
│   └── bimanual_demo.launch.py       # Bimanual sequential pick-and-place
├── config/
│   └── mtc.rviz                      # RViz config with Motion Planning Tasks panel
└── CMakeLists.txt
```

---

## Prerequisites

Two workspaces must both be sourced:

```bash
# MoveIt Task Constructor (separate workspace)
source ~/ws_moveit2/install/setup.bash

# This project
source ~/ros/ros2_ws/install/setup.bash
```

Build:

```bash
cd ~/ros/ros2_ws
colcon build --packages-select openarm_mtc_pick_place
source install/setup.bash
```

`yaml-cpp` is required for trajectory saving:

```bash
sudo apt install libyaml-cpp-dev
```

---

## Nodes

### 1. Single-arm pick-and-place

Picks a cylindrical bar with the right arm and places it at a nearby pose.

```bash
# Launch full stack
ros2 launch openarm_mtc_pick_place mtc_bringup.launch.py arm_type:=v1.0

# Run the task (separate terminal)
ros2 launch openarm_mtc_pick_place pick_place_demo.launch.py
```

### 2. Air pick-and-place (motion/gripper test)

Runs the full pick-and-place choreography using only relative moves and named gripper states — no collision objects or grasp-pose sampling. Use this to verify controller connectivity and motion scaling before introducing objects.

```bash
ros2 launch openarm_mtc_pick_place mtc_bringup.launch.py arm_type:=v1.0
ros2 launch openarm_mtc_pick_place air_pickplace_demo.launch.py
```

### 3. Bimanual handover

Right arm picks the bar from its original position, places it near the centerline; left arm then picks the same object from wherever right left it and places it on its own side. Single shared `stick` object — GenerateGraspPose reads its live scene pose automatically.

```bash
ros2 launch openarm_mtc_pick_place mtc_bringup.launch.py arm_type:=v1.0
ros2 launch openarm_mtc_pick_place handover_demo.launch.py
```

### 4. Bimanual sequential pick-and-place

Right and left arms each pick and place their own object sequentially in one MTC task chain (avoids MTC Merger + Generator stage limitation).

```bash
ros2 launch openarm_mtc_pick_place mtc_bringup.launch.py arm_type:=v1.0
ros2 launch openarm_mtc_pick_place parallel_demo.launch.py
```

---

## Trajectory Save & Playback

Every node automatically saves the planned trajectory to YAML before executing. Files are written to `/tmp/openarm_trajectories/` by default (one file per MTC stage).

**Override save directory at runtime:**

```bash
ros2 launch openarm_mtc_pick_place pick_place_demo.launch.py \
  trajectory_save_dir:=/home/ahsan/saved_trajs/run_01
```

**Check saved files:**

```bash
ls /tmp/openarm_trajectories/
# openarm_traj_stage_0.yaml   ← CurrentState (empty, skip)
# openarm_traj_stage_1.yaml   ← hands_up
# openarm_traj_stage_2.yaml   ← open gripper
# openarm_traj_stage_3.yaml   ← move to pick
# ...
```

**Find your controller names:**

```bash
ros2 control list_controllers
# right_joint_trajectory_controller   active
# right_gripper_controller            active
# left_joint_trajectory_controller    active
# left_gripper_controller             active
```

**Replay a single stage on real hardware:**

```bash
ros2 run openarm_mtc_pick_place playback_trajectory.py \
  /tmp/openarm_trajectories/openarm_traj_stage_5.yaml \
  right_joint_trajectory_controller
```

**Replay all stages in order (shell script):**

```bash
#!/bin/bash
TRAJ_DIR="/tmp/openarm_trajectories"
CONTROLLER="right_joint_trajectory_controller"

for yaml in $(ls $TRAJ_DIR/*.yaml | sort -V); do
  echo "Playing: $yaml"
  ros2 run openarm_mtc_pick_place playback_trajectory.py "$yaml" "$CONTROLLER"
  sleep 0.5
done
```

> Stages with empty `joint_names` or `points` (ModifyPlanningScene, CurrentState) are skipped automatically by the playback script.

---

## RViz Setup

After launching `mtc_bringup`, open RViz and add the **Motion Planning Tasks** panel:

- Set **Task Solution Topic** → `/solution`
- Save the config to `config/mtc.rviz` so it persists across restarts

To verify grasp pose orientation, enable **TF → Frames → openarm_right_hand** and check which axis points out of the fingertips before tuning `grasp_frame_transform`.

---

## Key Parameters

| Parameter | Location | Description |
|---|---|---|
| `arm_type` | launch arg | `v1.0` or `v2.0` — selects URDF/SRDF |
| `trajectory_save_dir` | ROS param | Directory for saved YAML stage files |
| `RIGHT_GRASP_Z_OFFSET` | `mtc_node.cpp` | Hand offset along Z from object at grasp |
| `RIGHT_GRASP_AXIS/ANGLE` | `mtc_node.cpp` | Rotation of hand frame relative to object |
| `setMaxVelocityScalingFactor` | all planners | Motion speed (0.1 conservative, 0.3 normal) |
| `setAngleDelta` | GenerateGraspPose | Grasp sample density (`M_PI/12` = 25 samples) |

---

## Notes

- The root frame is `openarm_body_link0` — all object poses and direction vectors use this frame, not `world`.
- MTC end-effector names must match SRDF `end_effector` entries: `right_ee` / `left_ee`, not the gripper group names.
- The `ExecuteTaskSolutionCapability` must be loaded in `move_group` — `mtc_bringup.launch.py` handles this; the shared `demo.launch.py` does not.
- On real hardware, all three planners (sampling, interpolation, Cartesian) need explicit velocity/acceleration scaling — without it, arm motion will be sudden at full speed.
