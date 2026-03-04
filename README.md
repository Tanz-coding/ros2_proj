# Autonomous CampusTurtlePost

基于 ROS 2 Humble + TurtleBot3 Burger + Gazebo 的校园快递配送仿真系统。

机器人在 Gazebo house 场景中自主导航到多个站点，途中遇障碍自动让行，到站拍照并记录 CSV 日志。

## 架构

```
┌─────────────────────────────────────────┐
│            数据记录层                     │
│   CSV 日志 · 到站拍照 · 输出目录          │
├─────────────────────────────────────────┤
│            任务层                         │
│   campus_delivery_node (多站点投递)       │
│   yield_controller_node (激光让行)        │
├─────────────────────────────────────────┤
│            基础能力层                     │
│   slam_toolbox (在线建图)                 │
│   Nav2 (路径规划 + 避障 + 行为树)         │
├─────────────────────────────────────────┤
│            仿真层                         │
│   Gazebo Server + TurtleBot3 Burger      │
│   激光雷达 /scan · 里程计 /odom           │
└─────────────────────────────────────────┘
```

详细设计文档：[docs/课程架构说明.md](docs/%E8%AF%BE%E7%A8%8B%E6%9E%B6%E6%9E%84%E8%AF%B4%E6%98%8E.md)

## 环境依赖

| 项目 | 版本 |
|------|------|
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| Gazebo | Classic 11 |
| Python | 3.10+ |

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-description \
  ros-humble-turtlebot3-msgs

pip3 install numpy scikit-learn
```

## 安装与构建

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Tanz-coding/CampusTurtlePost.git ros2

cd ~/ros2_ws
colcon build --packages-select autonomous_tb --symlink-install
source install/setup.bash
```

## 快速启动

**一条命令启动全部（推荐）：**

```bash
bash ~/ros2_ws/src/ros2/scripts/start_project.sh --rviz
```

脚本会自动完成：清理旧进程 → 加载环境 → 启动 Gazebo Server → 生成机器人 → 等待传感器就绪 → 启动 Nav2 + SLAM + 任务节点 → 打开 RViz。

### 启动选项

| 命令 | 说明 |
|------|------|
| `bash start_project.sh` | 无 GUI，纯后台仿真 + 任务 |
| `bash start_project.sh --rviz` | 带 RViz 可视化（**推荐**） |
| `bash start_project.sh --gui` | 带 Gazebo GUI（VM 中可能卡顿） |

### RViz 可视化说明

使用 `--rviz` 启动后，RViz 会自动加载预设配置，显示：

| 显示项 | 颜色 | 含义 |
|--------|------|------|
| Map | 灰白色 | slam_toolbox 实时建的地图 |
| LaserScan | 红色点 | 激光雷达扫描点云 |
| Global Path | 绿色线 | Nav2 全局规划路径 |
| Local Path | 黄色线 | 局部控制器跟踪路径 |
| RobotModel | 3D 模型 | 机器人当前位置 |

> 如果 RViz 卡顿，可在左侧面板取消勾选 **TF** 和 **RobotModel** 以提升帧率。

## 工作流程

```
启动 → slam_toolbox 在线建图 → Nav2 就绪
  → campus_delivery_node 发送 waypoint 1 (library)
    → Nav2 规划路径 → 机器人移动
    → yield_controller 监测 /scan，障碍 < 0.45m 暂停，> 0.60m 恢复
    → 到达 → 拍照 → 写 CSV
  → 发送 waypoint 2 (canteen) → ...
  → 发送 waypoint 3 (dormitory) → ...
  → 所有站点完成，打印 "All waypoints completed."
```

## 参数配置

配置文件：`config/campus_delivery_params.yaml`

```yaml
waypoint_names: ["library", "canteen", "dormitory"]
waypoint_x: [-1.0, 0.5, 1.8]
waypoint_y: [1.2, -0.8, 0.2]
waypoint_yaw: [0.0, 1.57, 3.14]
stop_distance: 0.45      # 障碍距离 < 此值暂停
resume_distance: 0.60    # 障碍距离 > 此值恢复
output_dir: "~/ros2_proj_data"
```

修改站点坐标后重新 `colcon build` 即可生效。

## 输出结果

任务完成后在 `~/ros2_proj_data/` 目录下生成：

```
~/ros2_proj_data/
├── logs/
│   └── delivery_log.csv    # 投递日志
└── photos/
    ├── library_20260303_223500.jpg
    ├── canteen_20260303_223520.jpg
    └── dormitory_20260303_223540.jpg
```

CSV 字段：`timestamp, waypoint, status, travel_time_sec, nearest_obstacle_m, photo_path`

## 项目结构

```
ros2/
├── autonomous_tb/                    # Python 节点
│   ├── campus_delivery_node.py       # 多站点投递任务
│   ├── yield_controller_node.py      # 激光让行控制
│   ├── exploration_node.py           # 自主探索（模式 A）
│   └── data_collection_node.py       # 数据采集
├── launch/
│   ├── nav2_mission.launch.py        # Nav2 + SLAM + 任务（主入口）
│   ├── campus_delivery.launch.py     # Gazebo + Nav2 + 任务（完整版）
│   ├── exploration.launch.py         # 自主探索模式
│   └── data_collection_launch.py     # 数据采集模式
├── config/
│   ├── nav2_params.yaml              # Nav2 导航参数
│   ├── campus_delivery_params.yaml   # 快递任务参数
│   └── nav2_view.rviz                # RViz 预设配置
├── scripts/
│   ├── start_project.sh              # 一键启动脚本
│   └── generate_blank_map.py         # 空白地图生成工具
├── maps/                             # 地图文件
├── docs/                             # 设计文档
├── setup.py
└── package.xml
```

## 常见问题

**Q: RViz 很卡？**
- 在 VMware 中：设置 → 显示器 → 勾选"加速 3D 图形" → 显存调到 256MB → 重启 VM
- 在 RViz 中取消勾选 TF 和 RobotModel

**Q: 机器人不动？**
- 检查终端是否打印 `Sending waypoint`，如果没有说明 Nav2 action server 未就绪
- 运行 `ros2 topic list` 确认有 `/scan` 和 `/odom`

**Q: 没有拍到照片？**
- TurtleBot3 Burger 默认没有相机，照片字段为空属正常
- 如需拍照功能，需在 model.sdf 中添加 camera 插件

**Q: "map frame does not exist"？**
- 启动后前几秒属正常现象，slam_toolbox 需要数帧 /scan 数据后才发布 map frame
- 如果持续报错，检查 /scan 话题是否有数据：`ros2 topic hz /scan`

## 演示素材

- [Gazebo_house.gif](Gazebo_house.gif)
- [house_map.png](house_map.png)
- [gazebo_world_map.png](gazebo_world_map.png)
