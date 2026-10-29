# apfrl_ws: APF-RL on a simulated TurtleBot3

A small ROS 2 workspace that runs the APF-RL navigation stack on a TurtleBot3 burger in
Gazebo. It contains no training code: the policy comes ready-trained.

**Platform:** Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (gz-sim 8).

## What is in it

| Package | What it does |
|---|---|
| `apfrl_pkgs/apf` | Artificial potential field. Turns a waypoint and a gain into velocity commands, steering around obstacles. |
| `apfrl_pkgs/obs_generator` | Groups the lidar scan into obstacles (`/scan` -> `/obstacles`) for the APF. |
| `apfrl_pkgs/apfrl` | The learned policy (`apfrl_node`). Three times a second, it reads the scan and the robot pose, then picks a waypoint (`/goal`) and an APF gain (`/apf_k`) for `apf_node`. |
| `apfrl_pkgs/apfrl_interfaces` | Messages, services and the `NavigateToPose` action. |
| `apfrl_pkgs/apfrl_rviz_plugins` | RViz display for `/obstacles`. |
| `simulation/apfrl_tb3_sim` | The launch file, a 20 x 20 m walled world (`worlds/dummy.world`), the RViz config, ground-truth odometry (`gt_odom_gz.py`) and an RViz goal bridge. |
| `dependencies/turtlebot3_description` | TurtleBot3 burger model, with gz_ros2_control. |
| `dependencies/zed-ros2-interfaces` | `zed_msgs`. `apfrl` needs it to compile. |

**The policy:** `apfrl/models/apfrl_dyn_model.pt` is a SAC policy (three hidden layers
of 512) trained for 1M steps in TurtleBot3 rooms with moving obstacles, exported to
TorchScript.

## How the nodes connect

```
gz lidar ─/scan─┬─> obs_generator ─/obstacles─> apf_node ─/diff_drive_controller/cmd_vel─> robot
                └─> apfrl_node ─/goal, /apf_k─────┘
gt_odom_gz ─/odom_gt─> apf_node, apfrl_node      (plus TF odom -> base_footprint)
RViz "2D Goal Pose" ─/goal_pose─> rviz_goal_bridge ─action /apfrl/navigate_to_pose─> apfrl_node
```

`odom` is the Gazebo world frame, so a goal at (3, 2) means world position (3, 2).

## One-time setup

1. **ROS packages:**
   ```bash
   sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
       ros-jazzy-ros2-controllers ros-jazzy-xacro python3-colcon-common-extensions
   ```
2. **CUDA and libtorch.** `apfrl_node` runs the policy with libtorch on the GPU.
   - Install the CUDA toolkit, with `nvcc` on `PATH`.
   - Install a libtorch build that matches your CUDA version (C++ ABI, cxx11), for
     example in `~/Libraries/libtorch`.
   - Then:
     ```bash
     echo 'export TORCH_DIR=$HOME/Libraries/libtorch' >> ~/.bashrc
     echo 'export PATH=$PATH:/usr/local/cuda/bin' >> ~/.bashrc
     echo $HOME/Libraries/libtorch/lib | sudo tee /etc/ld.so.conf.d/libtorch.conf && sudo ldconfig
     ```
   - Building and running without CUDA is not supported: `apfrl/CMakeLists.txt` requires it.
3. **No conda or micromamba environment active** while you build, not even `(base)`.
   - CMake would pick up that Python, which lacks ROS's build modules.
   - The symptom is `ModuleNotFoundError: No module named 'lark'` in `apfrl_interfaces`.
   - Run `conda deactivate` / `micromamba deactivate` first, then delete `build/` and
     `install/` before rebuilding: the wrong Python is cached in `build/`.

## Build

```bash
cd apfrl_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## Run

```bash
source apfrl_ws/install/setup.bash
ros2 launch apfrl_tb3_sim apfrl_tb3.launch.py
```

- **What starts:** Gazebo, then the robot at the origin. The planners follow after
  about 8 s, and RViz opens.
- **To set a goal:** in RViz, click **2D Goal Pose** and drag a goal anywhere inside
  the walls. The robot drives there, and a new click replaces the goal.
- **Without RViz**, send a goal from the command line:
  ```bash
  ros2 action send_goal /apfrl/navigate_to_pose apfrl_interfaces/action/NavigateToPose \
    "{target_pose: {header: {frame_id: odom}, pose: {position: {x: 5.0, y: 3.0}, orientation: {w: 1.0}}}}" \
    --feedback
  ```
- **Launch arguments** (`--show-args` lists them):
  - `world:=/path/to/file.world`
  - `x:=`, `y:=`, `yaw:=` set the spawn pose.
  - `headless:=true` starts no Gazebo window.
  - `rviz:=false`
  - `model_path:=/path/to/policy.pt`

**Writing your own world:**
- Start from `dummy.world`.
- Name the world `default`: `gt_odom_gz` reads `/world/default/dynamic_pose/info`.
- Keep the `gz-sim-sensors-system` plugin, or the lidar publishes nothing.

**Do not change the `apf` and `apfrl` parameters in the launch file:**
- The policy was trained with exactly these: speed limits 0.8 / -0.3 m/s and pi/3 rad/s,
  robot radius 0.4 m, stable gradient, 3 Hz, `obs_profile: legacy`, ground-truth odometry.
- Change them and the policy's actions no longer mean what it learned, so it drives
  worse.

## Useful topics

| Topic | What it carries |
|---|---|
| `/scan` | Lidar |
| `/obstacles` | Obstacles found by `obs_generator` |
| `/goal` | The policy's current waypoint |
| `/apf_k` | The policy's current APF gain |
| `/odom_gt` | Ground-truth pose |
| `/diff_drive_controller/odom` | Wheel odometry, which drifts |
| `/navigation_status` | The planner's state |
