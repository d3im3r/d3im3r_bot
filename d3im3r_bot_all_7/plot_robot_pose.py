#!/usr/bin/env python3
import math
import threading
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32


class RobotPosePlotter(Node):
    def __init__(self) -> None:
        super().__init__('robot_pose_plotter')

        # Parámetros físicos
        self.declare_parameter('wheel_radius_m', 0.0217)
        self.declare_parameter('wheel_base_m', 0.10144)
        self.declare_parameter('use_imu_heading', True)
        self.declare_parameter('history_len', 5000)

        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self.wheel_base_m = float(self.get_parameter('wheel_base_m').value)
        self.use_imu_heading = bool(self.get_parameter('use_imu_heading').value)
        self.history_len = int(self.get_parameter('history_len').value)

        # Estado
        self.left_w_rad_s = 0.0
        self.right_w_rad_s = 0.0
        self.imu_yaw_rad = 0.0
        self.have_imu = False
        self.have_wheels = False

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()

        self.path_x = deque(maxlen=self.history_len)
        self.path_y = deque(maxlen=self.history_len)
        self.path_x.append(self.x)
        self.path_y.append(self.y)

        self.lock = threading.Lock()

        self.create_subscription(
            Vector3,
            '/wheel_velocities_rad_s',
            self.wheel_vel_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/imu_yaw_rad',
            self.imu_yaw_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.update_pose)

        self.get_logger().info('RobotPosePlotter iniciado.')

    def wheel_vel_callback(self, msg: Vector3) -> None:
        with self.lock:
            self.left_w_rad_s = float(msg.x)
            self.right_w_rad_s = float(msg.y)
            self.have_wheels = True

    def imu_yaw_callback(self, msg: Float32) -> None:
        with self.lock:
            self.imu_yaw_rad = self.wrap_to_pi(float(msg.data))
            self.have_imu = True

    def update_pose(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 0.2:
            return

        with self.lock:
            if not self.have_wheels:
                return

            wl = self.left_w_rad_s
            wr = self.right_w_rad_s

            # Velocidades lineales de rueda
            vl = wl * self.wheel_radius_m
            vr = wr * self.wheel_radius_m

            # Velocidad lineal del robot
            v = 0.5 * (vr + vl)

            # Heading
            if self.use_imu_heading and self.have_imu:
                self.theta = self.imu_yaw_rad
            else:
                omega = (vr - vl) / self.wheel_base_m
                self.theta = self.wrap_to_pi(self.theta + omega * dt)

            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt

            self.path_x.append(self.x)
            self.path_y.append(self.y)

    @staticmethod
    def wrap_to_pi(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main() -> None:
    rclpy.init()
    node = RobotPosePlotter()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, ax = plt.subplots(figsize=(7, 7))
    line_traj, = ax.plot([], [], linewidth=2, label='Trayectoria')
    point_robot, = ax.plot([], [], marker='o', markersize=8, label='Robot')
    heading_line, = ax.plot([], [], linewidth=2, label='Orientación')

    ax.set_title('Trayectoria estimada del robot')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.axis('equal')
    ax.grid(True)
    ax.legend()

    def animate(_frame):
        with node.lock:
            xs = list(node.path_x)
            ys = list(node.path_y)
            x = node.x
            y = node.y
            theta = node.theta

        if not xs or not ys:
            return line_traj, point_robot, heading_line

        line_traj.set_data(xs, ys)
        point_robot.set_data([x], [y])

        arrow_len = 0.08
        hx = x + arrow_len * math.cos(theta)
        hy = y + arrow_len * math.sin(theta)
        heading_line.set_data([x, hx], [y, hy])

        margin = 0.2
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)

        if abs(xmax - xmin) < 0.2:
            xmin -= 0.1
            xmax += 0.1
        if abs(ymax - ymin) < 0.2:
            ymin -= 0.1
            ymax += 0.1

        ax.set_xlim(xmin - margin, xmax + margin)
        ax.set_ylim(ymin - margin, ymax + margin)

        return line_traj, point_robot, heading_line

    ani = FuncAnimation(fig, animate, interval=50, blit=False)

    try:
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()