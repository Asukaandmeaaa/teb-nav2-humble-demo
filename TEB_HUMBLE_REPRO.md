# ROS2 Humble + Nav2 + TEB Local Planner Demo

这个工作区是我基于 `ROS2 Humble`、`Nav2` 和 `teb_local_planner` 搭建的一套局部规划仿真工程。  
它的目标很直接：在 TurtleBot3 仿真环境中，把 `TEB` 作为 `Nav2 controller_server` 的局部控制器跑起来，并完成地图、定位、导航和局部避障这一整条链路的联调。

这份文档按 README 的方式整理，主要说明这个工作区已经实现了什么、目录里关键内容在哪里，以及别人拿到工程之后该怎么复现。

## 项目实现了什么

目前这个工作区已经完成了下面几件事：

- 在 `ROS2 Humble` 环境下成功编译 ROS2 版 `teb_local_planner`
- 将 `teb_local_planner::TebLocalPlannerROS` 挂接到 `Nav2 controller_server`
- 基于 TurtleBot3 + Gazebo 建立了一套最小可运行的仿真流程
- 提供了独立的 launch 和参数文件，避免直接修改 `/opt/ros/humble` 下的系统配置
- 跑通了 `map_server`、`amcl`、`planner_server`、`controller_server`、`bt_navigator` 这一整套导航链路
- 在 RViz 中完成 `2D Pose Estimate` 后，可以建立 `map -> odom` TF，并进入 `Navigation: active`、`Localization: active` 状态

换句话说，这个工作区实现的不是单独“把 TEB 源码放进来”，而是已经把 TEB 真正接进了 Nav2，并能在仿真里作为局部规划器工作。

## 系统环境

我当前验证通过的环境是：

- Ubuntu 22.04
- ROS2 Humble
- Navigation2
- turtlebot3_navigation2
- turtlebot3_gazebo

工作区中包含的关键源码有：

- `src/teb_local_planner`
- `src/costmap_converter`
- `src/teb_nav2_demo`

## 项目结构

这个工程里和复现最相关的部分主要是下面这些：

- `src/teb_local_planner`
  `teb_local_planner` 本体源码
- `src/costmap_converter`
  TEB 依赖的 `costmap_converter`
- `src/teb_nav2_demo`
  我额外补的最小 demo 包
- `src/teb_nav2_demo/launch/teb_tb3_sim.launch.py`
  一键启动仿真和导航的 launch 文件
- `src/teb_nav2_demo/params/teb_burger.yaml`
  Nav2 + TEB 的核心参数文件

## 核心思路

我参考的是 ROS1 Melodic 的 TEB 教程。  
ROS1 教程里的实现链路一般是：

- `Gazebo / TurtleBot3`
- `map_server + amcl`
- `move_base`
- `teb_local_planner`

迁移到我当前环境之后，对应关系变成了：

- `Gazebo / TurtleBot3`
- `map_server + amcl`
- `Nav2`
- `controller_server` 中加载 `teb_local_planner::TebLocalPlannerROS`

这里最关键的一点，就是把 TEB 从 ROS1 里的 `move_base` 局部规划器，迁移成 ROS2 里 `Nav2 controller_server` 的 controller plugin。

## 关键配置

这个工程能跑起来，最关键的配置在 `src/teb_nav2_demo/params/teb_burger.yaml`。

其中最核心的是这一段：

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "teb_local_planner::TebLocalPlannerROS"
```

这表示我已经把 Nav2 默认的局部控制器替换成了 TEB。

另外这份参数文件里还配置了：

- TurtleBot3 的速度和加速度约束
- TEB 的轨迹采样与优化参数
- 同伦类搜索参数
- 局部和全局 costmap 参数
- `costmap_converter` 插件

## 我做过的兼容修补

为了让这份 ROS2 版 TEB 能在 Humble 下真正编译和运行，我做过几处兼容处理，主要包括：

- 修复 `costmap_converter` 中 `cv_bridge` 头文件路径不兼容的问题
- 修复部分旧式日志接口调用
- 修复 `teb_local_planner` 对旧版 Nav2 异常头文件的引用
- 修复 `nav2_costmap_2d::calculateMinAndMaxDistances()` 的 Humble 接口差异
- 修复 demo 参数文件中的 `static_layer.plugin` 缺失问题
- 修复 `teb_autosize` 参数类型不匹配问题
- 修改 launch，使其根据 `TURTLEBOT3_MODEL` 自动选择正确的 Gazebo 模型

这些处理之后，下面这些包已经能在当前工作区中成功编译：

- `costmap_converter_msgs`
- `costmap_converter`
- `teb_msgs`
- `teb_local_planner`
- `teb_nav2_demo`

## 如何复现

### 1. 编译工作区

```bash
cd ~/teb_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/teb_ws/install/setup.bash
```

### 2. 设置 TurtleBot3 型号

这个 demo 默认按 `burger` 来复现：

```bash
export TURTLEBOT3_MODEL=burger
```

### 3. 启动仿真

```bash
cd ~/teb_ws
source /opt/ros/humble/setup.bash
source ~/teb_ws/install/setup.bash
ros2 launch teb_nav2_demo teb_tb3_sim.launch.py
```

这个 launch 会自动完成下面几件事：

- 启动 Gazebo
- 载入 TurtleBot3 模型
- 启动 Nav2
- 使用 `teb_burger.yaml` 作为导航参数文件
- 打开 RViz

### 4. 在 RViz 中完成定位

启动之后，不要直接发导航目标，先做两步：

1. 点击 `2D Pose Estimate`
2. 在地图中设置机器人初始位姿

这一步完成后，正常情况下会建立 `map -> odom` TF，RViz 左下角会逐渐进入：

- `Navigation: active`
- `Localization: active`

### 5. 发送导航目标

定位完成之后：

1. 点击 `Nav2 Goal`
2. 在地图中选择一个目标点
3. 观察机器人是否开始执行全局路径规划和局部避障

## 效果展示

这一部分建议在上传 GitHub 前补上运行截图，方便别人快速了解工程状态。

### 1. 导航系统成功激活

建议放一张 RViz 截图，画面中最好能看到：

- `Navigation: active`
- `Localization: active`
- `Global Status: Ok`

图注示例：

> 图 1  Nav2 成功启动后，地图、定位和导航状态均正常。

[此处插入截图：RViz 激活状态]

### 2. 初始位姿设置完成

建议放一张完成 `2D Pose Estimate` 之后的截图，能够体现：

- 机器人当前位姿已经与地图对齐
- `map -> odom` 变换已经建立

图注示例：

> 图 2  完成初始位姿设置后，定位链路建立，机器人进入可导航状态。

[此处插入截图：初始位姿设置完成]

### 3. 发出导航目标后的规划效果

建议放一张发送 `Nav2 Goal` 后的截图，最好能同时体现：

- 全局路径
- 局部轨迹
- 机器人朝目标点运动

图注示例：

> 图 3  发送目标点后，Nav2 完成全局路径规划，TEB 负责局部轨迹优化与避障控制。

[此处插入截图：全局路径与局部轨迹]

### 4. TEB 插件成功加载的终端日志

如果想让 README 更完整，还可以补一张终端截图，保留类似下面的关键信息：

```text
Created controller : FollowPath of type teb_local_planner::TebLocalPlannerROS
```

图注示例：

> 图 4  `controller_server` 成功加载 `teb_local_planner::TebLocalPlannerROS`。

[此处插入截图：TEB 插件加载日志]

## 如何确认当前用的确实是 TEB

可以通过下面几种方式确认：

### 方法 1：检查 controller 插件参数

```bash
source /opt/ros/humble/setup.bash
source ~/teb_ws/install/setup.bash
ros2 param get /controller_server FollowPath.plugin
```

正常应该返回：

```bash
teb_local_planner::TebLocalPlannerROS
```

### 方法 2：看启动日志

在启动终端里，正常会看到类似日志：

```text
Created controller : FollowPath of type teb_local_planner::TebLocalPlannerROS
```

### 方法 3：观察局部轨迹行为

在 RViz 中发出目标点之后，可以观察：

- 全局路径是否生成
- 局部轨迹是否连续更新
- 机器人是否出现局部避障和轨迹重规划行为

## 建议的验证命令

如果启动后想快速确认导航链路是否正常，可以执行：

```bash
ros2 node list | sort
ros2 action info /navigate_to_pose
ros2 run tf2_ros tf2_echo map odom
```

理想情况下应该满足：

- 存在 `/controller_server`
- 存在 `/planner_server`
- 存在 `/bt_navigator`
- 存在 `/map_server`
- 存在 `/amcl`
- `/navigate_to_pose` 的 action server 是 `/bt_navigator`
- `map -> odom` 可以正常查询到

## 常见问题

### 1. RViz 中提示 `Fixed Frame [map] does not exist`

这通常不是 map 文件缺失，而是定位还没完成。  
先确认：

- `amcl` 是否启动
- 是否执行了 `2D Pose Estimate`
- `map -> odom` 是否已经建立

### 2. `Nav2 Goal` 发不出去，或者 action server 不可用

优先检查：

- `/bt_navigator` 是否存在
- `/navigate_to_pose` 是否有 action server
- `nav2_container` 是否崩溃

### 3. Gazebo 提示 entity already exists

这是旧仿真进程没有清干净。可以先清理：

```bash
pkill -f gzserver
pkill -f gzclient
pkill -f rviz2
pkill -f component_container_isolated
pkill -f "ros2 launch teb_nav2_demo"
```

再重新启动。

### 4. `nav2_container` 崩溃

我这边实际遇到过的一个典型原因是参数文件里漏了：

```yaml
static_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
```

如果这个字段缺失，`global_costmap` 在配置时会直接报错退出。

## 后续可以继续做什么

在当前基础上，后面还可以继续做这些工作：

- 调整 TEB 参数，观察局部轨迹变化
- 更换地图或障碍物场景，测试不同局部避障效果
- 对比 TEB 和 DWB 的局部规划表现
- 整理为更正式的实验报告或 GitHub 项目主页

## 总结

这个工作区目前已经不只是“包含一份 TEB 源码”，而是已经完成了从源码适配、Nav2 接入到仿真联调的完整流程。  
如果后续我要把它整理上传到 GitHub，这个工程已经具备比较完整的 README 结构和可复现入口，后面只需要继续补截图、补结果展示，以及补一部分参数说明，就可以作为一个相对完整的公开项目来维护。
