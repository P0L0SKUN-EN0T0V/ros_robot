#!/bin/bash
# Перезапуск всех нод проекта одной командой.
# Убивает старые процессы → запускает launch (Ctrl+C валит все три ноды).

set -e

# 1. Прибиваем старые ноды (если висят с прошлого запуска)
pkill -f planner.launch.py 2>/dev/null || true
pkill -f fake_sim_node     2>/dev/null || true
pkill -f global_planner_node 2>/dev/null || true
pkill -f web_viz_node      2>/dev/null || true
sleep 1

# 2. Окружение ROS2
cd "$(dirname "$0")/ros2_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 3. Опциональные пути логов (нужны на VPS с read-only $HOME)
if [ ! -w "$HOME/.ros" ] 2>/dev/null; then
    export ROS_LOG_DIR=/tmp/ros_log
    export ROS_HOME=/tmp/ros_home
fi

# 4. exec — launch заменяет shell, Ctrl+C идёт прямо в него
#    и корректно валит всю группу нод
exec ros2 launch global_planner planner.launch.py
