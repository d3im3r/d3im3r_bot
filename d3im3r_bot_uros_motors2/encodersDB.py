#!/usr/bin/env python3

import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from pynput import keyboard


class PPRRecorder(Node):

    def __init__(self):
        super().__init__("ppr_recorder")

        self.left_count = 0
        self.right_count = 0

        self.left_file = "ppr_left.csv"
        self.right_file = "ppr_right.csv"

        self._init_file(self.left_file)
        self._init_file(self.right_file)

        self.sub_left = self.create_subscription(
            Int32,
            "/left_ticks",
            self.left_callback,
            10
        )

        self.sub_right = self.create_subscription(
            Int32,
            "/right_ticks",
            self.right_callback,
            10
        )

        self.pub_clear_left = self.create_publisher(Int32, "/load_count_left", 10)
        self.pub_clear_right = self.create_publisher(Int32, "/load_count_right", 10)

        print("\nControls:")
        print("l -> store LEFT")
        print("r -> store RIGHT")
        print("c -> clear both")
        print("q -> quit\n")

    def _init_file(self, filename):
        try:
            with open(filename, "x", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp", "pulses"])
        except FileExistsError:
            pass

    def left_callback(self, msg):
        self.left_count = msg.data

    def right_callback(self, msg):
        self.right_count = msg.data

    def _append(self, filename, value):

        timestamp = datetime.now().isoformat()

        with open(filename, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, value])

    def reset_left(self):
        msg = Int32()
        msg.data = 0
        self.pub_clear_left.publish(msg)

    def reset_right(self):
        msg = Int32()
        msg.data = 0
        self.pub_clear_right.publish(msg)

    def save_left(self):
        self._append(self.left_file, self.left_count)
        print("LEFT stored:", self.left_count)
        self.reset_left()

    def save_right(self):
        self._append(self.right_file, self.right_count)
        print("RIGHT stored:", self.right_count)
        self.reset_right()

    def clear(self):
        self.reset_left()
        self.reset_right()
        print("Counters cleared")


def main():

    rclpy.init()
    node = PPRRecorder()

    running = True

    def on_press(key):
        nonlocal running

        try:
            if key.char == "l":
                node.save_left()

            elif key.char == "r":
                node.save_right()

            elif key.char == "c":
                node.clear()

            elif key.char == "q":
                running = False
                return False

        except AttributeError:
            pass

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    try:
        while rclpy.ok() and running:
            rclpy.spin_once(node, timeout_sec=0.1)

    finally:
        listener.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()