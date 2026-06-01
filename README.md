# ROS 2 Navigation Sandbox

Автономная навигация робота в 2D-лабиринте: log-odds occupancy mapping, Theta\* path-planning, Pure Pursuit, recovery state-machine. Без GPU и без Gazebo — на любом headless VPS или локалке через Docker.

Стек: **ROS 2 Jazzy** (Ubuntu 24.04) · C++ planner · Python симулятор · Foxglove Studio для визуализации через `foxglove_bridge`.

---

## Запуск (3 варианта на выбор)

### 1. Docker на локалке (Windows / Mac / Linux)

Требуется Docker Desktop (Win/Mac) или `docker` + `docker compose` (Linux).

```bash
git clone https://github.com/P0L0SKUN-EN0T0V/ros_robot.git
cd ros_robot
docker compose up -d --build
docker logs ros_robot      # проверь что все 3 ноды стартовали
```

Открой Foxglove Studio → **Open connection → Foxglove WebSocket** → `ws://localhost:8765` → Open.

В Foxglove → **Layouts → Import from file** → `foxglove_layout.json` из репо.

### 2. Docker на VPS (для удалённой работы)

То же самое, но клонируешь на VPS:

```bash
git clone https://github.com/P0L0SKUN-EN0T0V/ros_robot.git
cd ros_robot
docker compose up -d --build
```

С локалки в Foxglove подключаешься напрямую:
- `ws://VPS_IP:8765` — если 8765 открыт в файрволе хостера и `ufw allow 8765/tcp`
- или через SSH-туннель: `ssh -L 8765:localhost:8765 user@VPS_IP`, потом в Foxglove `ws://localhost:8765`

### 3. Нативный запуск (без Docker, на Ubuntu 24.04)

```bash
# Один раз — установка ROS 2 Jazzy и зависимостей
sudo apt update
sudo apt install ros-jazzy-ros-base \
                 ros-jazzy-foxglove-bridge \
                 ros-jazzy-tf2-geometry-msgs \
                 ros-jazzy-tf2-ros

# Сборка
cd ros_robot/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select global_planner --symlink-install

# Запуск (одна команда — стартует все три ноды)
./run.sh
```

Скрипт `run.sh` сам убивает прошлые процессы, source-ит окружение и делает `ros2 launch global_planner planner.launch.py`. Ctrl+C валит всю группу.

---

## Что внутри

- **`fake_sim_node.py`** — 2D-симулятор: робот, 720-лучевой лидар, одометрия, лабиринт из 19 толстых стен (10 см). Замена Gazebo для VPS без GPU. Публикует `/scan`, `/odom`, `/tf`, `/walls_real` (ground truth для второй панели Foxglove), `/odom_trail`.
- **`global_planner_node.cpp`** — маппинг (log-odds, Bresenham) + **Theta\*** (any-angle pathfinding) + Pure Pursuit + recovery state-machine (backup + replan при контакте). Карта 400×400 @ 2.5 см.
- **`planner.launch.py`** — стартует все три ноды плюс `foxglove_bridge` на :8765.
- **`foxglove_layout.json`** — готовый layout: две 3D-панели (Robot view + Reality), включённые топики, publish на `/goal_pose` по клику.

---

## Топики

| Топик | Тип | Кто публикует | Описание |
|---|---|---|---|
| `/scan` | LaserScan | fake_sim | Лидар (720 лучей, range_max=3.5 м) |
| `/odom` | Odometry | fake_sim | Позиция робота |
| `/tf`, `/tf_static` | TFMessage | fake_sim | odom→base_link, base_link→base_scan |
| `/walls_real` | MarkerArray | fake_sim | Ground truth стен лабиринта (для Foxglove) |
| `/odom_trail` | Path | fake_sim | История позиций робота |
| `/map` | OccupancyGrid | planner | Накопленная карта от лидара |
| `/global_plan` | Path | planner | Текущий путь Theta\* |
| `/cmd_vel` | Twist | planner | Команды управления |
| `/goal_pose` | PoseStamped | client | Цель навигации (Foxglove click) |
| `/move_base_simple/goal` | PoseStamped | client | Альтернативный legacy-топик goal-а |

---

## Архитектура алгоритмов

### Маппинг — Log-Odds Occupancy Grid

Каждая клетка хранит float `log_odds` ∈ [-2.0, +3.5]:
- Луч лидара прошёл сквозь клетку → `+= -0.4` (free)
- Луч хитнул в клетку → `+= +0.85` (occupied)

Преимущество перед бинарной картой `{0, 100}`: один ложный hit не фиксирует стену навсегда. При публикации `/map` конвертим через пороги: `> +0.4` = occupied, `< -0.4` = free, иначе unknown.

**Без FOV-decay** (раньше был, убран) — для статичной сцены он давал зацикливание (робот забывал стену → A\* строил путь через неё → contact). Карта только растёт.

### Path planning — Theta\*

Расширение A\* (any-angle): при relax-е соседа `N` пробуем привязать его не к текущему узлу, а к **родителю текущего** (бабушке), если между ними есть line-of-sight (Bresenham + isFree). Это даёт прямые пути под любым углом, не ступенчатые.

8-направленная сетка остаётся для топологии открытого списка, но parent-указатели могут «прыгать» через клетки.

### Контроллер — Pure Pursuit + двухфазная стратегия

- `MAX_LINEAR_VEL = 0.5 м/с`, `LOOKAHEAD_DIST = 0.3 м`
- Если `|alpha| > 0.4 rad` — только крутимся на месте (linear=0)
- Иначе linear = MAX × (1 − |alpha|/0.4 × 0.5) — лёгкое замедление на повороте
- Replan каждые 5 control-тиков (500 мс), Pure Pursuit — каждый тик (100 мс)

### Recovery — state-machine

```
NORMAL → (контакт <0.18 м) → BACKING_UP → (отъехал 30 см) → NORMAL
```

В BACKING_UP робот едет строго назад (`-0.1 м/с`, angular=0), Pure Pursuit не вызывается. После отъезда — пересчёт inflated map + replan на свежей карте.

### TF

`fake_sim` публикует:
- dynamic `odom → base_link` (20 Hz)
- static `base_link → base_scan`

Без этого Foxglove/RViz не умеют рендерить `/scan` в кадре `odom`.

---

## Параметры

| Параметр | Значение | Где |
|---|---|---|
| MAP_SIZE | 400×400 | global_planner_node.cpp |
| RESOLUTION | 0.025 м (2.5 см/cell) | global_planner_node.cpp |
| INFLATION_RADIUS | 10 cells = 0.25 м | global_planner_node.cpp |
| LOG_ODDS_FREE / OCC | −0.4 / +0.85 | global_planner_node.cpp |
| MAX_LINEAR_VEL | 0.5 м/с | global_planner_node.cpp |
| MAX_ANGULAR_VEL | 1.5 rad/с | global_planner_node.cpp |
| ALIGN_THRESHOLD | 0.4 rad | global_planner_node.cpp |
| EMERGENCY_TRIGGER | 0.18 м | global_planner_node.cpp |
| BACKUP_DISTANCE | 0.3 м | global_planner_node.cpp |
| Лидар лучи | 720 (0.5° между) | fake_sim_node.py |
| Лидар range_max | 3.5 м | fake_sim_node.py |
| Толщина стен | 0.10 м | fake_sim_node.py |

---

## Структура

```
ros_robot/
├── Dockerfile                  # ros:jazzy + наши пакеты + colcon build
├── docker-compose.yml          # один сервис, проброс :8765
├── .dockerignore               # исключения для COPY
├── .gitattributes              # eol=lf для всех текстовых (от CRLF на Windows)
├── .gitignore
├── run.sh                      # нативный запуск (без Docker)
├── foxglove_layout.json        # готовый layout для Foxglove Studio
├── docs/                       # учебный туториал (статичный HTML, отдельная история)
└── ros2_ws/
    └── src/global_planner/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/planner.launch.py
        └── src/
            ├── global_planner_node.cpp
            └── fake_sim_node.py
```

---

## Подводные камни

### Ubuntu 24.04 → ROS 2 Jazzy (не Humble)

Humble только для 22.04. На 24.04 (Noble) — Jazzy:
```bash
sudo apt install ros-jazzy-ros-base
```

### Gazebo не работает на headless VPS

Gazebo требует GPU/дисплей. Используем `fake_sim_node.py` — 2D-симулятор на Python через ray-segment intersection.

### ROS_LOG_DIR / ROS_HOME если $HOME read-only

`run.sh` уже выставляет в `/tmp` если `~/.ros` недоступен. В Dockerfile тоже.

### CRLF на Windows ломает Python shebang

Git с настройкой `autocrlf=true` (default на Windows) конвертирует LF→CRLF при clone. В shebang `#!/usr/bin/env python3` появляется `\r` → `/usr/bin/env: 'python3\r': No such file`.

Защита есть на двух уровнях:
- `.gitattributes` в репо принудительно использует `eol=lf` для `.py/.sh/etc`
- Dockerfile дополнительно делает `sed -i 's/\r$//'` после COPY

### Executable bit на .py теряется на Windows

При clone Windows не сохраняет unix permissions. Dockerfile делает `chmod +x` после COPY.

### foxglove_bridge ≠ rosbridge

Foxglove из коробки предлагает `rosbridge_server`, но он **ломает publishing PoseStamped в ROS 2** (несовместимый header). Использовать **`foxglove_bridge`** (apt-пакет `ros-jazzy-foxglove-bridge`) — родной мост Foxglove, бинарный протокол, корректный publishing.

### Foxglove default publish topic — `/move_base_simple/goal`, не `/goal_pose`

Planner подписан на оба для совместимости. Можно поменять в Foxglove → 3D panel → Publish → 2D pose → Topic = `/goal_pose`.

### rclcpp::Time с разными ClockType бросает std::runtime_error

`rclcpp::Time` без инициализации создаётся с `RCL_SYSTEM_TIME`, а `get_clock()->now()` возвращает `RCL_ROS_TIME`. Вычитание падает. Использовать счётчик тиков для интервалов, не `rclcpp::Time`.

---

## Использование

В Foxglove после подключения и импорта layout:

1. **Robot view (левая панель)** — то что робот «знает»: карта `/map`, лидар `/scan`, путь `/global_plan`, trail.
2. **Reality (правая)** — ground truth: стены `/walls_real` как синие коробки, тот же trail, та же траектория плана.

В 3D-панели справа найди иконку **Publish Pose** (маркер/мишень в toolbar) → клик и протяг по полу = отправка `/goal_pose`. Робот строит Theta\* план и едет.

При контакте со стеной в логе появится:
```
[WARN]  Obstacle 0.16 m → recovery: backing up 30 cm and replanning
[INFO]  Recovery done (0.30 m back), replanned: N poses
```

---

## Полезные команды

```bash
# Запуск
docker compose up -d --build      # первая сборка + запуск (~5-10 мин первый раз)
docker compose up -d              # просто запуск (после первого билда)
docker compose logs -f            # логи в реальном времени
docker compose down               # остановить и удалить контейнер
docker compose restart            # перезапуск

# Нативно
./run.sh                          # запуск (или перезапуск, убьёт прошлое)

# Отладка
docker exec -it ros_robot bash    # внутрь контейнера
ros2 topic list                   # все топики
ros2 topic echo /odom --once      # одно сообщение топика
ros2 topic hz /scan               # частота публикации
```
