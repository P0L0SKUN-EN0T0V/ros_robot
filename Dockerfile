# Базовый образ ROS 2 Jazzy (Ubuntu 24.04 + ros-base)
FROM ros:jazzy-ros-base

# Системные зависимости + ROS пакеты которые мы используем
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Сначала только package.xml — Docker кеширует слой с deps,
# и при изменении кода re-install пакетов не делается
COPY ros2_ws/src/global_planner/package.xml ./ros2_ws/src/global_planner/

# Теперь весь src
COPY ros2_ws/src ./ros2_ws/src

# Сборка через colcon (как и на хосте)
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    cd ros2_ws && \
    colcon build --packages-select global_planner --symlink-install"

# Foxglove WebSocket порт
EXPOSE 8765

# Логи в /tmp — внутри контейнера $HOME может быть read-only
ENV ROS_LOG_DIR=/tmp/ros_log
ENV ROS_HOME=/tmp/ros_home

# Запуск всех нод одной launch-командой
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && \
                    source /workspace/ros2_ws/install/setup.bash && \
                    ros2 launch global_planner planner.launch.py"]
