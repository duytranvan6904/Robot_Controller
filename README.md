# Yaskawa HC10DTP Real-Time Cartesian Control

This project provides a real-time Cartesian streaming pipeline for the collaborative **Yaskawa HC10DTP** robot using MotoROS2's Point Queue Mode. It receives Cartesian coordinates (from AI or Camera nodes), solves Inverse Kinematics (IK) at high frequency (e.g., 20Hz-25Hz), and streams joint angles directly to MotoROS2.

## 📦 Required Packages

- **OS:** Ubuntu 22.04 LTS
- **ROS 2:** Humble Hawksbill
- **Required Packages:**
  - `moveit2` (for IK solving via the `/compute_ik` service)
  - `motoros2_interfaces` (for MotoROS2 queue mode services like `StartPointQueueMode` and `QueueTrajPoint`)
  - `hc10dtp_moveit_config` (for URDF models, RViz, and MoveIt configurations)
  - `hc10dtp_bringup` (contains the `cartesian_streamer_hc10dtp.py` pipeline)

---

## 🚀 How to Run the Real-Time Control Project

To operate the robot using the streaming pipeline, you need to open two separate terminals.

> **⚠️ CRITICAL ARCHITECTURE NOTE:** 
> Do **NOT** run `real_robot.launch.py` (if it exists). That launch file would start `ros2_control` and a joint trajectory controller, which creates a conflicting secondary command path to the robot. The MotoROS2 driver runs directly on the YRC1000micro controller and exposes `/yaskawa/*` services automatically.

### Terminal 1: Start MoveIt & State Publishers
Launch the MoveIt group, RViz, and robot state publishers specifically for HC10DTP:

```bash
ros2 launch hc10dtp_moveit_config hc10dtp_start.launch.py
```

### Terminal 2: Start the Cartesian Streamer
Run the Python streaming controller for HC10DTP. You can test the integration by running it with built-in demo patterns:

```bash
cd ~/gp4_ws/src/hc10dtp_bringup/scripts
python3 cartesian_streamer_hc10dtp.py --demo line
```

**Available Demo Patterns:**
- `python3 cartesian_streamer_hc10dtp.py --demo line` (Moves back and forth along the X-axis)
- `python3 cartesian_streamer_hc10dtp.py --demo circle` (Draws a circular trajectory)
- `python3 cartesian_streamer_hc10dtp.py --demo lissajous` (Draws a Lissajous curve)

---

## 📡 Integration (AI / Camera Nodes)

If you run the streamer without a demo argument, it will wait for commands from your external AI or Camera tracking node.

**Send commands via:**
- `/cartesian_streamer/target_pose` (`geometry_msgs/PoseStamped`)
- `/cartesian_streamer/target_xyz` (`std_msgs/Float64MultiArray` with `[x, y, z]`. Orientation will be maintained from the current pose).

**Read feedback from:**
- `/cartesian_streamer/current_pose` (`geometry_msgs/PoseStamped`) — Publishes the current smoothed End-Effector position.
