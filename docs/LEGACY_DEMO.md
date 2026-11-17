# teb-nav2-humble-demo

ROS 2 Humble workspace for running the TEB local planner with Nav2.

This repository packages a working Nav2 + TEB demo, including the ROS 2 version of
`teb_local_planner`, `costmap_converter`, a custom `teb_nav2_demo` package, maps,
robot descriptions, Gazebo world files, and launch files for validation.

## What This Project Does

- Builds `teb_local_planner` on ROS 2 Humble.
- Loads `teb_local_planner::TebLocalPlannerROS` as a Nav2 controller plugin.
- Provides a ready-to-run Nav2 simulation using Gazebo, RViz, map server, AMCL,
  planner server, controller server, and BT navigator.
- Includes a planning-only launch file for quickly testing global planning on a
  static map without starting Gazebo.
- Records debug outputs under `debug/teb_tb3_sim_latest/` during simulation runs.

## Tested Environment

- Ubuntu 22.04
- ROS 2 Humble
- Nav2
- Gazebo Classic
- colcon

## Repository Layout

```text
src/
  costmap_converter/      TEB dependency used for obstacle conversion
  teb_local_planner/      ROS 2 TEB local planner source
  teb_nav2_demo/          Demo package with launch files, maps, params, URDFs
  maps/                   Additional map assets

TEB_HUMBLE_REPRO.md       Detailed reproduction and debugging notes
```

The most important demo files are:

- `src/teb_nav2_demo/launch/teb_tb3_sim.launch.py`
- `src/teb_nav2_demo/launch/teb_map_planning.launch.py`
- `src/teb_nav2_demo/params/teb_burger.yaml`
- `src/teb_nav2_demo/maps/WHEELTEC.yaml`
- `src/teb_nav2_demo/worlds/WHEELTEC.world`

## Install Dependencies

Install ROS 2 Humble first, then install common Nav2 and simulation packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-tf2-tools \
  python3-colcon-common-extensions \
  python3-rosdep
```

Initialize `rosdep` if it has not been initialized on your machine:

```bash
sudo rosdep init
rosdep update
```

Then install workspace dependencies:

```bash
cd teb-nav2-humble-demo
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
cd teb-nav2-humble-demo
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run the Gazebo + Nav2 + TEB Demo

Start the full simulation:

```bash
cd teb-nav2-humble-demo
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch teb_nav2_demo teb_tb3_sim.launch.py
```

By default this launch file:

- starts Gazebo in headless mode,
- spawns the demo robot,
- loads the `WHEELTEC` map,
- starts Nav2,
- uses TEB as the controller plugin,
- opens RViz,
- writes debug snapshots to `debug/teb_tb3_sim_latest/`.

To show the Gazebo client window:

```bash
ros2 launch teb_nav2_demo teb_tb3_sim.launch.py headless:=false
```

In RViz, send a goal with **Nav2 Goal** and watch the global path, local plan,
and robot motion.

## Run Planning-Only Mode

For quick global-planner checks without Gazebo:

```bash
cd teb-nav2-humble-demo
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch teb_nav2_demo teb_map_planning.launch.py
```

You can request a path with:

```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
"{use_start: true, planner_id: GridBased, start: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, goal: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

## Key Nav2 Configuration

The controller plugin is configured in
`src/teb_nav2_demo/params/teb_burger.yaml`:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "teb_local_planner::TebLocalPlannerROS"
```

That is the main integration point where Nav2 loads TEB as the local planner.

## Notes

- Build outputs are intentionally ignored: `build/`, `install/`, `log/`, and
  `debug/` are not tracked by Git.
- The original upstream sources for `teb_local_planner` and `costmap_converter`
  came from the ROS 2 branches of the `rst-tu-dortmund` repositories, with local
  compatibility changes for this Humble demo.
- See `TEB_HUMBLE_REPRO.md` for detailed reproduction notes, known fixes, and
  debugging context.

## License

This workspace contains third-party packages with their original licenses. Check
the package-level license files before redistributing modified versions.
