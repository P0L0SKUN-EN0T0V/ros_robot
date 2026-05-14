# ROS2 A* Navigation — Пошаговая сборка на VPS

Проект: автономная навигация робота с A* pathfinding, собранный с нуля на headless VPS.

## Что внутри

- **fake_sim_node.py** — 2D симулятор (замена Gazebo для VPS без GPU)
- **global_planner_node.cpp** — маппинг (Bresenham) + A* + Pure Pursuit + emergency stop
- **planner.launch.py** — запуск обоих нод одной командой

## Быстрый старт

```bash
# 1. Установка ROS2 (Ubuntu 24.04 = Jazzy, НЕ Humble!)
sudo apt install ros-jazzy-ros-base ros-jazzy-tf2-geometry-msgs

# 2. Сборка
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select global_planner

# 3. Запуск
export ROS_LOG_DIR=/tmp/ros_log
export ROS_HOME=/tmp/ros_home
source install/setup.bash
ros2 launch global_planner planner.launch.py

# 4. Отправить цель (в другом терминале)
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}" \
  -w 1 --times 3
```

## Топики

| Топик | Тип | Описание |
|---|---|---|
| `/scan` | LaserScan | Лидар (360 лучей, fake_sim) |
| `/odom` | Odometry | Позиция робота |
| `/cmd_vel` | Twist | Команды скорости |
| `/map` | OccupancyGrid | Карта 200x200, 5 см/ячейка |
| `/global_plan` | Path | Путь от A* |
| `/goal_pose` | PoseStamped | Цель навигации |

## Подводные камни (pitfalls)

### 1. Ubuntu 24.04 → ROS2 Jazzy, НЕ Humble

```
Unable to locate package ros-humble-ros-base
```

Humble поддерживает только Ubuntu 22.04. На 24.04 (Noble) используй **Jazzy**:
```bash
sudo apt install ros-jazzy-ros-base
```

### 2. Gazebo не работает на headless VPS

Gazebo требует GPU и дисплей. На VPS без GPU используем **fake_sim_node.py** —
лёгкий 2D симулятор на Python: лидар через ray-segment intersection, одометрия,
коллизия со стенами.

### 3. ROS_LOG_DIR и ROS_HOME

```
Failed to create log directory '/home/user/.ros/log'
```

На VPS домашняя директория может быть read-only (tmpfs, контейнеры). Решение:
```bash
export ROS_LOG_DIR=/tmp/ros_log
export ROS_HOME=/tmp/ros_home
```
Добавь в `.bashrc` или в launch file.

### 4. std::vector brace-init — narrowing error

```cpp
// ОШИБКА: компилятор думает что 40000 это narrowing в int8_t
std::vector<int8_t> map_data_{MAP_SIZE * MAP_SIZE, -1};

// ПРАВИЛЬНО: явный вызов конструктора
std::vector<int8_t> map_data_ = std::vector<int8_t>(MAP_SIZE * MAP_SIZE, -1);
```

### 5. Lambda + auto в ROS2 Jazzy

```cpp
// ОШИБКА: Jazzy не может разрешить тип через auto
scan_sub_ = create_subscription<...>("/scan", 10, [this](auto msg){...});

// ПРАВИЛЬНО: явный тип
scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg){...});
```

### 6. tf2::fromMsg — undefined reference

```
undefined reference to 'tf2::fromMsg'
```

Нужен пакет `tf2_geometry_msgs`:
```cmake
# CMakeLists.txt
find_package(tf2_geometry_msgs REQUIRED)
ament_target_dependencies(... tf2_geometry_msgs)
```
```xml
<!-- package.xml -->
<depend>tf2_geometry_msgs</depend>
```
```cpp
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
```

### 7. ros2 topic pub --once не доходит

```bash
# ПРОБЛЕМА: --once выходит до того, как subscriber подключится
ros2 topic pub /goal_pose ... --once  # subscriber может не успеть

# РЕШЕНИЕ: ждём subscriber + шлём несколько раз
ros2 topic pub /goal_pose ... -w 1 --times 3
```

### 8. sudo на VPS — пароль, не NOPASSWD

Если VPS настроен другим пользователем и пароль неизвестен:
1. Зайди в панель хостинга (DigitalOcean → Droplet → Access)
2. Reset Root Password → получишь пароль на email
3. `su - root`
4. `echo "username ALL=(ALL:ALL) ALL" > /etc/sudoers.d/username && chmod 440 /etc/sudoers.d/username`

**Важно**: используй `ALL=(ALL:ALL) ALL` (с паролем), НЕ `NOPASSWD:ALL`.
Безопасность > удобство.

## Этапы сборки

Проект собирался поэтапно, каждый этап — отдельная проверка:

1. **Скелет** — пустая нода, colcon build, "started" в логе
2. **Fake Sim** — /scan (360 лучей) и /odom публикуются
3. **Маппинг** — Bresenham ray tracing, /map: free=9708, wall=331
4. **A*** — planAStar, goal (2.0, 1.0) → путь 41 ячейка
5. **Pure Pursuit** — робот доехал: "Goal reached! pos=(1.87, 0.96)"
6. **Финал** — emergency stop, launch file, README

## Параметры

| Параметр | Значение | Описание |
|---|---|---|
| MAP_SIZE | 200x200 | Размер карты в ячейках |
| RESOLUTION | 0.05 м | Размер ячейки |
| LOOKAHEAD_DIST | 0.3 м | Pure Pursuit lookahead |
| MAX_LINEAR_VEL | 0.15 м/с | Макс. линейная скорость |
| GOAL_TOLERANCE | 0.15 м | Допуск достижения цели |
| Emergency stop | 0.20 м | Мин. дистанция до препятствия |

## Структура

```
ros2_ws/
└── src/global_planner/
    ├── CMakeLists.txt
    ├── package.xml
    ├── global_planner/__init__.py
    ├── launch/planner.launch.py
    └── src/
        ├── global_planner_node.cpp
        └── fake_sim_node.py
```
