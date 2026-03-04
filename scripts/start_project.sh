#!/usr/bin/env bash
set -euo pipefail

USE_GUI="false"
USE_RVIZ="false"
if [[ "${1:-}" == "--gui" ]]; then
  USE_GUI="true"
elif [[ "${1:-}" == "--rviz" ]]; then
  USE_RVIZ="true"
fi

echo "[1/7] 清理旧进程..."
pkill -f gzclient || true
pkill -f gzserver || true
pkill -f gazebo || true
pkill -f rviz2 || true

echo "[2/7] 加载 ROS 环境..."
set +u
source /opt/ros/humble/setup.bash

if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  source "$HOME/ros2_ws/install/setup.bash"
  echo "[环境] 使用工作区: $HOME/ros2_ws"
elif [[ -f "$HOME/ros2_ws/src/install/setup.bash" ]]; then
  source "$HOME/ros2_ws/src/install/setup.bash"
  echo "[环境] 使用工作区: $HOME/ros2_ws/src"
else
  set -u
  echo "[错误] 未找到 install/setup.bash，请先在你的工作区执行 colcon build"
  exit 1
fi
set -u
export TURTLEBOT3_MODEL=burger
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
export MESA_GL_VERSION_OVERRIDE=3.3
export OGRE_RTT_MODE=Copy

TB3_PREFIX="$(ros2 pkg prefix turtlebot3_gazebo)"
WORLD_FILE="$TB3_PREFIX/share/turtlebot3_gazebo/worlds/turtlebot3_house.world"
MODEL_FILE="$TB3_PREFIX/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf"

echo "[3/7] 启动 Gazebo Server..."
ros2 launch gazebo_ros gzserver.launch.py world:="$WORLD_FILE" verbose:=false > /tmp/tb3_gzserver.log 2>&1 &
GZSERVER_PID=$!

echo "[4/7] 启动 robot_state_publisher..."
ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=true > /tmp/tb3_rsp.log 2>&1 &
RSP_PID=$!

echo "[5/7] 等待 /spawn_entity 服务..."
for _ in {1..40}; do
  if ros2 service list | grep -q "/spawn_entity"; then
    break
  fi
  echo "  - /spawn_entity 未就绪，继续等待..."
  sleep 0.5
done

if ! ros2 service list | grep -q "/spawn_entity"; then
  echo "[错误] /spawn_entity 服务未就绪，请查看 /tmp/tb3_gzserver.log"
  kill "$GZSERVER_PID" "$RSP_PID" 2>/dev/null || true
  exit 1
fi

echo "[6/7] 生成 TurtleBot3 实体..."
ros2 run gazebo_ros spawn_entity.py -entity tb3 -file "$MODEL_FILE" -x -2.0 -y 1.25 > /tmp/tb3_spawn.log 2>&1 || {
  echo "[错误] 机器人生成失败，请查看 /tmp/tb3_spawn.log"
  kill "$GZSERVER_PID" "$RSP_PID" 2>/dev/null || true
  exit 1
}

echo "[7/7] 等待 /scan 与 /odom 话题..."
for _ in {1..60}; do
  TOPICS="$(ros2 topic list)"
  if echo "$TOPICS" | grep -q "^/scan$" && echo "$TOPICS" | grep -q "^/odom$"; then
    echo "[OK] 传感器和里程计话题已就绪"
    break
  fi
  echo "  - 话题未就绪，当前只检测到: $(echo "$TOPICS" | tr '\n' ' ' | sed 's/  */ /g')"
  sleep 0.5
done

TOPICS="$(ros2 topic list)"
if ! echo "$TOPICS" | grep -q "^/scan$" || ! echo "$TOPICS" | grep -q "^/odom$"; then
  echo "[错误] /scan 或 /odom 未出现，请查看 /tmp/tb3_*.log"
  kill "$GZSERVER_PID" "$RSP_PID" 2>/dev/null || true
  exit 1
fi

if [[ "$USE_GUI" == "true" ]]; then
  echo "[可选] 启动 Gazebo GUI（VM 低负载模式）..."
  gzclient 2>/tmp/tb3_gzclient.log &
  echo "  提示: 如果 GUI 卡顿，可关闭 GUI 窗口，仿真不受影响"
fi

if [[ "$USE_RVIZ" == "true" ]]; then
  echo "[可选] 启动 RViz 可视化..."
  PKG_SHARE_TB="$(ros2 pkg prefix autonomous_tb)/share/autonomous_tb"
  RVIZ_CFG="$PKG_SHARE_TB/config/nav2_view.rviz"
  if [[ -f "$RVIZ_CFG" ]]; then
    rviz2 -d "$RVIZ_CFG" 2>/tmp/tb3_rviz.log &
  else
    rviz2 2>/tmp/tb3_rviz.log &
    echo "  提示: 首次使用请在 RViz 中手动添加显示项（见 README）"
  fi
fi

# ---------- 检查地图文件 ----------
PKG_SHARE="$(ros2 pkg prefix autonomous_tb)/share/autonomous_tb"
MAP_YAML="$PKG_SHARE/maps/map.yaml"
MAP_PGM="$PKG_SHARE/maps/map.pgm"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f "$MAP_PGM" ]]; then
  echo "[注意] 未找到地图文件 $MAP_PGM，生成空白地图..."
  python3 "$SCRIPT_DIR/generate_blank_map.py" "$(dirname "$MAP_YAML")"
fi

echo "[启动] Nav2 + 任务节点..."
exec ros2 launch autonomous_tb nav2_mission.launch.py
