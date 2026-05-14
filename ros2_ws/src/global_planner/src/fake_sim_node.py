#!/usr/bin/env python3
"""
Fake 2D Simulator — замена Gazebo для VPS без GPU.

Публикует /scan (LaserScan) и /odom (Odometry).
Принимает /cmd_vel (Twist) и двигает виртуального робота.
Карта — простая комната с препятствиями.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import math
import time


class FakeSim(Node):
    def __init__(self):
        super().__init__('fake_sim_node')

        # === Карта: список стен (отрезков) ===
        # Формат: (x1, y1, x2, y2)
        self.walls = [
            # Внешние стены комнаты 8x8 (от -4 до 4)
            (-4, -4,  4, -4),  # нижняя
            ( 4, -4,  4,  4),  # правая
            ( 4,  4, -4,  4),  # верхняя
            (-4,  4, -4, -4),  # левая
            # Внутренние препятствия
            (-1.5, -1.5, -1.5,  0.5),  # вертикальная стена слева
            ( 1.0, -2.5,  1.0, -0.5),  # вертикальная стена справа
            ( 0.5,  2.0,  2.5,  2.0),  # горизонтальная полка сверху
        ]

        # === Состояние робота ===
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0

        # === LiDAR параметры (как у TurtleBot3 burger) ===
        self.num_rays = 360
        self.angle_min = 0.0
        self.angle_max = 2 * math.pi
        self.range_min = 0.12
        self.range_max = 3.5

        # === ROS2 ===
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Таймер: 20 Hz обновление
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info(
            f'Fake Sim started: robot at ({self.x:.1f}, {self.y:.1f}), '
            f'{len(self.walls)} walls')

    def cmd_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    def update(self):
        # 1. Обновляем позицию робота (простая кинематика)
        self.yaw += self.wz * self.dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        new_x = self.x + self.vx * math.cos(self.yaw) * self.dt
        new_y = self.y + self.vx * math.sin(self.yaw) * self.dt

        # Sub-stepping вдоль траектории — иначе на скорости робот может
        # за один тик «прошить» тонкую стену (tunneling)
        if not self._path_collides(self.x, self.y, new_x, new_y):
            self.x = new_x
            self.y = new_y

        # 2. Публикуем одометрию
        self.publish_odom()

        # 3. Публикуем лидар
        self.publish_scan()

    def _collides(self, x, y, radius=0.15):
        """Проверка столкновения точки (x,y) со стенами."""
        for (x1, y1, x2, y2) in self.walls:
            dist = self._point_to_segment_dist(x, y, x1, y1, x2, y2)
            if dist < radius:
                return True
        return False

    def _path_collides(self, x0, y0, x1, y1, radius=0.15):
        """Проверка коллизии вдоль отрезка движения с sub-stepping."""
        dx = x1 - x0
        dy = y1 - y0
        step_len = math.hypot(dx, dy)
        if step_len < 1e-9:
            return self._collides(x1, y1, radius)
        n = max(2, int(math.ceil(step_len / (radius * 0.5))))
        for i in range(n + 1):
            t = i / n
            if self._collides(x0 + t * dx, y0 + t * dy, radius):
                return True
        return False

    def _point_to_segment_dist(self, px, py, x1, y1, x2, y2):
        """Расстояние от точки до отрезка."""
        dx = x2 - x1
        dy = y2 - y1
        len_sq = dx * dx + dy * dy
        if len_sq < 1e-10:
            return math.hypot(px - x1, py - y1)
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / len_sq))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        return math.hypot(px - proj_x, py - proj_y)

    def _ray_segment_intersect(self, ox, oy, dx, dy, x1, y1, x2, y2):
        """Пересечение луча (ox,oy)+t*(dx,dy) с отрезком. Возвращает t или None."""
        sx = x2 - x1
        sy = y2 - y1
        denom = dx * sy - dy * sx
        if abs(denom) < 1e-10:
            return None
        t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
        if t > 0 and 0 <= u <= 1:
            return t
        return None

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_scan'
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = (self.angle_max - self.angle_min) / self.num_rays
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.time_increment = 0.0
        msg.scan_time = self.dt

        ranges = []
        for i in range(self.num_rays):
            angle = self.yaw + self.angle_min + i * msg.angle_increment
            ray_dx = math.cos(angle)
            ray_dy = math.sin(angle)

            min_dist = self.range_max
            for (x1, y1, x2, y2) in self.walls:
                t = self._ray_segment_intersect(
                    self.x, self.y, ray_dx, ray_dy, x1, y1, x2, y2)
                if t is not None and t < min_dist:
                    min_dist = t

            if min_dist < self.range_min:
                min_dist = self.range_min
            ranges.append(min_dist)

        msg.ranges = ranges
        self.scan_pub.publish(msg)

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y

        # Yaw → quaternion
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2)

        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.angular.z = self.wz

        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
