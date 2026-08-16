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


class D3im3rPosePlotter(Node):
    def __init__(self) -> None:
        super().__init__('d3im3r_pose_plotter')

        self.declare_parameter('wheel_radius_m', 0.0217)
        self.declare_parameter('history_len', 4000)
        self.declare_parameter('use_imu_heading', True)

        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self.history_len = int(self.get_parameter('history_len').value)
        self.use_imu_heading = bool(self.get_parameter('use_imu_heading').value)

        self.left_w = 0.0
        self.right_w = 0.0
        self.yaw = 0.0

        self.have_wheels = False
        self.have_yaw = False

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
            self.wheel_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/imu_yaw_rad',
            self.yaw_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.update_pose)

        self.get_logger().info('Nodo de trayectoria iniciado.')

    def wheel_callback(self, msg: Vector3) -> None:
        with self.lock:
            self.left_w = float(msg.x)
            self.right_w = float(msg.y)
            self.have_wheels = True

    def yaw_callback(self, msg: Float32) -> None:
        with self.lock:
            self.yaw = self.wrap_to_pi(float(msg.data))
            self.have_yaw = True

    def reset_pose(self) -> None:
        with self.lock:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.path_x.clear()
            self.path_y.clear()
            self.path_x.append(self.x)
            self.path_y.append(self.y)

    def update_pose(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 0.2:
            return

        with self.lock:
            if not self.have_wheels:
                return

            wl = self.left_w
            wr = self.right_w

            vl = wl * self.wheel_radius_m
            vr = wr * self.wheel_radius_m
            v = 0.5 * (vl + vr)

            if self.use_imu_heading and self.have_yaw:
                self.theta = self.yaw

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
    node = D3im3rPosePlotter()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, ax = plt.subplots(figsize=(8, 8))
    traj_line, = ax.plot([], [], linewidth=2, label='Trayectoria')
    robot_point, = ax.plot([], [], marker='o', markersize=8, label='Robot')
    heading_line, = ax.plot([], [], linewidth=2, label='Heading')

    ax.set_title('Trayectoria estimada del robot')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.grid(True)
    ax.axis('equal')
    ax.legend()

    def on_key(event):
        if event.key == 'r':
            node.reset_pose()

    fig.canvas.mpl_connect('key_press_event', on_key)

    def animate(_):
        with node.lock:
            xs = list(node.path_x)
            ys = list(node.path_y)
            x = node.x
            y = node.y
            theta = node.theta
            wl = node.left_w
            wr = node.right_w

        if len(xs) == 0:
            return traj_line, robot_point, heading_line

        traj_line.set_data(xs, ys)
        robot_point.set_data([x], [y])

        arrow_len = 0.08
        hx = x + arrow_len * math.cos(theta)
        hy = y + arrow_len * math.sin(theta)
        heading_line.set_data([x, hx], [y, hy])

        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)

        margin = 0.20
        if abs(xmax - xmin) < 0.3:
            xmin -= 0.15
            xmax += 0.15
        if abs(ymax - ymin) < 0.3:
            ymin -= 0.15
            ymax += 0.15

        ax.set_xlim(xmin - margin, xmax + margin)
        ax.set_ylim(ymin - margin, ymax + margin)

        ax.set_title(
            f'Trayectoria estimada | wl={wl:.2f} rad/s | wr={wr:.2f} rad/s | yaw={theta:.2f} rad'
        )

        return traj_line, robot_point, heading_line

    FuncAnimation(fig, animate, interval=50, blit=False)

    try:
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()