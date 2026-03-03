# Autonomous TurtleBot

本项目基于 ROS 2 Humble，支持两类功能：

- 自主探索建图（Cartographer + Nav2 + Frontier Exploration）
- 校园快递任务（多站点投递 + 激光让行 + 到站拍照 + CSV 记录）

适配场景：TurtleBot3 + Gazebo 仿真环境。

## 1. 功能概览

### 1.1 自主探索建图

- 启动 Gazebo 仿真环境
- 使用 Cartographer 实时建图
- 使用 Nav2 进行路径规划与避障
- 自定义探索节点自动搜索 frontier 区域

### 1.2 校园快递任务（MVP）

- 按站点顺序自动导航投递（`NavigateToPose`）
- 订阅 `/scan` 实现让行：障碍过近暂停，安全后恢复
- 每到一站自动拍照（默认话题 `/camera/image_raw`）
- 自动生成任务日志 CSV（状态、耗时、最近障碍距离、照片路径）

## 2. 环境依赖

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo（Classic）
- TurtleBot3 相关包
- Nav2（`navigation2`、`nav2_bringup`）
- Cartographer（`cartographer_ros`）
- Python 3.8+
- Python 库：`numpy`、`scikit-learn`

安装示例：

```bash
sudo apt update
sudo apt install -y \
	ros-humble-cartographer \
	ros-humble-cartographer-ros \
	ros-humble-navigation2 \
	ros-humble-nav2-bringup \
	ros-humble-turtlebot3-gazebo

pip3 install numpy scikit-learn
```

## 3. 安装与构建

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Tanz-coding/ros2_proj.git

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 4. 使用方式

### 4.1 模式 A：自主探索建图

启动：

```bash
ros2 launch autonomous_tb exploration.launch.py
```

说明：

- 会启动 Gazebo + Cartographer + Nav2 + 探索节点
- 可在 RViz 观察地图、轨迹、frontier 标记（`frontiers`）

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 4.2 模式 B：校园快递任务

启动：

```bash
ros2 launch autonomous_tb campus_delivery.launch.py
```

运行步骤：

1. 启动后在 RViz 使用 `2D Pose Estimate` 设置初始位姿
2. 节点将按 `waypoints` 自动逐点投递
3. 遇到近距离障碍会自动暂停，障碍移开后继续

## 5. 参数配置（校园快递）

配置文件：[config/campus_delivery_params.yaml](config/campus_delivery_params.yaml)

重点参数：

- `waypoints`：站点列表（`name/x/y/yaw`）
- `stop_distance`：触发暂停阈值（米）
- `resume_distance`：恢复阈值（米）
- `image_topic`：图像话题
- `output_dir`：输出目录

## 6. 输出结果

- 照片：`~/ros2_proj_data/photos/*.jpg`
- 日志：`~/ros2_proj_data/logs/delivery_log.csv`

CSV 字段：

- `timestamp`
- `waypoint`
- `status`
- `travel_time_sec`
- `nearest_obstacle_m`
- `photo_path`

## 7. 项目结构

- [autonomous_tb/exploration_node.py](autonomous_tb/exploration_node.py)：自主探索主节点
- [autonomous_tb/data_collection_node.py](autonomous_tb/data_collection_node.py)：图像/激光数据采集节点
- [autonomous_tb/campus_delivery_node.py](autonomous_tb/campus_delivery_node.py)：校园快递任务节点
- [autonomous_tb/yield_controller_node.py](autonomous_tb/yield_controller_node.py)：让行控制节点
- [launch/exploration.launch.py](launch/exploration.launch.py)：探索模式启动文件
- [launch/data_collection_launch.py](launch/data_collection_launch.py)：数据采集启动文件
- [launch/campus_delivery.launch.py](launch/campus_delivery.launch.py)：快递任务启动文件
- [config/nav2_params.yaml](config/nav2_params.yaml)：Nav2 参数
- [config/campus_delivery_params.yaml](config/campus_delivery_params.yaml)：快递任务参数

## 8. 常见问题

- 没有图像话题：先执行 `ros2 topic list | grep image` 检查相机话题是否存在。
- 机器人不动：确认 Nav2 已启动，且在 RViz 设置了初始位姿。
- 频繁暂停：适当增大 `resume_distance`，并避免 `stop_distance` 与 `resume_distance` 过于接近。

## 9. 演示素材

- [Gazebo_house.gif](Gazebo_house.gif)
- [house_map.png](house_map.png)
- [gazebo_world_map.png](gazebo_world_map.png)
