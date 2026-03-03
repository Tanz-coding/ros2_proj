# ROS2 校园快递投递 MVP

基于 **ROS 2 Humble + Nav2 + slam_toolbox + Gazebo Classic + TurtleBot3 Burger** 的最小可运行项目。

## 功能覆盖

- Gazebo 仿真中的 TurtleBot3 多站点自动投递
- `/scan` 触发让行逻辑：近距离障碍时暂停，安全后恢复
- 到站自动拍照（订阅图像话题，保存 JPG）
- 到站结果记录到 CSV

## 工作区结构

```text
ros2/
  src/
    campus_delivery_mission/
      campus_delivery_mission/
      config/
      launch/
      resource/
      package.xml
      setup.py
      setup.cfg
```

## 前置依赖

- Ubuntu 22.04
- ROS 2 Humble
- Nav2（`nav2_bringup`）
- `slam_toolbox`
- `gazebo_ros_pkgs`
- TurtleBot3 Burger 仿真包
- Python 依赖：`opencv-python`、`cv_bridge`

可用以下命令检查关键包：

```bash
ros2 pkg list | egrep "nav2|slam_toolbox|gazebo_ros|turtlebot3"
```

## 构建

在工作区根目录（本仓库）：

```bash
colcon build --symlink-install
source install/setup.bash
```

## 运行流程

### 1) 启动仿真（示例，按你的 TB3 安装方式调整）

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 2) 建图并保存（一次性）

```bash
ros2 launch slam_toolbox online_async_launch.py
```

机器人覆盖环境后保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/campus_map
```

### 3) 加载地图并启动 Nav2

```bash
ros2 launch nav2_bringup bringup_launch.py map:=~/maps/campus_map.yaml use_sim_time:=True
```

在 RViz2 中设置 `2D Pose Estimate`。

### 4) 启动任务节点

```bash
source install/setup.bash
ros2 launch campus_delivery_mission mission.launch.py
```

## 可调参数

默认参数在 [src/campus_delivery_mission/config/waypoints.yaml](src/campus_delivery_mission/config/waypoints.yaml)。

- `waypoints`: 投递点（`name/x/y/yaw`）
- `image_topic`: 相机话题
- `scan_topic`: 激光话题
- `stop_distance`: 暂停阈值（米）
- `resume_distance`: 恢复阈值（米）
- `output_dir`: 输出目录（照片和 CSV）

## 输出结果

- 照片：`~/ros2_proj_data/photos/*.jpg`
- 日志：`~/ros2_proj_data/logs/delivery_log.csv`

CSV 字段：

- `timestamp`
- `waypoint`
- `status`
- `travel_time_sec`
- `nearest_obstacle_m`
- `photo_path`

## 常见问题

- 没有图像话题：执行 `ros2 topic list | grep image`，并确认仿真模型启用了相机。
- 无法导航：确认 Nav2 已激活，且 AMCL 初始位姿已设置。
- 频繁暂停：适当增大 `stop_distance` 或减小 `resume_distance` 抖动。
