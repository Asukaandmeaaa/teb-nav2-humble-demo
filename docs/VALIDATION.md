# 公开模型验证记录

日期：2026-11-17。仅指本仓库的 `generic_demo_robot`，不使用私有实车模型。
ROS 2 Humble / Gazebo Classic；Docker 容器中编译并运行，GUI 开启、Mesa 软件渲染 2 线程。
本次在已有 Humble 依赖镜像中完成源码构建；新增 Dockerfile 的从零镜像安装未在本轮重复执行。

## 完整走廊测试

原始运行：`trial_20261117_105932_teb_UTC`。
启动 `./scripts/sim.sh gui`，另一终端执行 `./scripts/sim.sh trial`。
默认参数：`long_corridor`、`responsive`、`publish_ground_truth=false`。

| 指标 | 实测 |
| --- | ---: |
| 动作状态 / 综合判定 | SUCCEEDED / passed=true |
| 总墙钟时间（含等待就绪） | 144.84 s |
| 通道通过 | 3 / 3 |
| 动态障碍触发及完成位移 | 2 / 2 |
| 实际物理接触 | 0 |
| 最终目标距离 | 0.173 m |
| 最小带裕量轮廓到静态障碍间距 | 0.100 m |
| 最小带裕量轮廓到动态障碍间距 | 0.224 m |
| 运动开始后低速停车累计 | 0 s（阈值 0.03 m/s） |
| TEB 内部计算 P95 | 42.35 ms |
| TEB 内部计算最大值 | 250.75 ms |

这是单次公开模型回归，不能解释为成功率或所有场景的实时保证。
计算长尾仍会超过 20 Hz 控制周期；P95 不是端到端避障响应时间。
场景几何、动作状态、采样指标与规划消息见 [公开报告](data/corridor_run.json)，
位姿轨迹见 [CSV](data/corridor_run.csv)。CSV 为 5 Hz 左右的显示采样，指标由完整原始采样计算。

记录 253 条全局规划和 273 条局部规划快照，坐标系均为 odom。
动图选取当前时刻之前最近的规划消息，不使用未来路径；路径最多保留约 250 个采样点。
机器人位置、障碍位置来自 Gazebo；这里只用于回放及独立评测。
导航仍只使用激光、TF、在线 SLAM 和仿真里程计，不读取该报告的真值。

## 程序检查

- `colcon build --symlink-install --packages-up-to dynamic_nav_sim`：6 个包完成。
- `python3 -m unittest discover -s src/dynamic_nav_sim/test -v`：8 项通过。
- 覆盖圆弧拟合/直线排除、运动-停止-重启跟踪、TF 等待/预测过期、自车回波 NaN，以及起点新鲜度。
- `check_urdf src/dynamic_nav_sim/urdf/generic_robot.urdf`：解析通过。
- Python 编译检查和 Shell 语法检查通过。
- Nav2 轮廓检查的额外回归用例随覆盖包保留在 `src/nav2_costmap_2d/test/unit/footprint_collision_checker_test.cpp`。

## 回放复现

```bash
python3 scripts/render_readme.py --trial docs/data/corridor_run.json --output results/replay
```

两段 GIF 均为实测数据可视化，不是 Gazebo 屏幕录像：

- `docs/media/corridor_planning.gif`：整段走廊，5 倍速。
- `docs/media/door_and_obstacle.gif`：第一门及红色动态障碍，2.5 倍速。

需要重新验证时，应启动新场景后再运行 trial；不能对到达终点的场景重复发送完整评测。
