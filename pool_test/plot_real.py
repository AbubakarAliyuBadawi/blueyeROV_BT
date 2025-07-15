#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.animation import FuncAnimation

def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

class LivePosePlot(Node):
    def __init__(self):
        super().__init__('live_pose_plot')

        self.times = []
        self.north = []
        self.east = []
        self.depth = []
        self.heading = []
        self.start_time = None

        # Subscriptions
        self.create_subscription(PoseStamped, '/blueye/pose', self.pose_callback, 10)
        self.create_subscription(Float32, '/blueye/depth', self.depth_callback, 10)

        # Setup matplotlib figure and axes
        self.fig, (self.ax_n, self.ax_e, self.ax_d, self.ax_h) = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
        self.ax_h.set_ylim(-np.pi - 0.5, np.pi + 0.5)
        self.ax_h.axhline(np.pi, linestyle='--', alpha=0.5)
        self.ax_h.axhline(-np.pi, linestyle='--', alpha=0.5)
        self.ax_h.set_xlabel('Elapsed Time (s)')

        # Start animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)

    def pose_callback(self, msg: PoseStamped):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = t
        elapsed = t - self.start_time

        x = msg.pose.position.x
        y = msg.pose.position.y

        q = msg.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        yaw_wrapped = wrap_angle(yaw)

        self.times.append(elapsed)
        self.east.append(x)
        self.north.append(y)
        self.heading.append(yaw_wrapped)

    def depth_callback(self, msg: Float32):
        # Just store the latest depth; it will be matched to the latest time
        self.depth.append(msg.data)

    def update_plot(self, frame):
        # Clear all axes
        for ax in (self.ax_n, self.ax_e, self.ax_d, self.ax_h):
            ax.cla()

        # Plot each component
        self.ax_n.plot(self.times, self.north, 'b-', linewidth=1)
        self.ax_n.set_ylabel('North (m)')
        self.ax_n.set_title('North Position')

        self.ax_e.plot(self.times, self.east, 'r-', linewidth=1)
        self.ax_e.set_ylabel('East (m)')
        self.ax_e.set_title('East Position')

        # Plot depth with proper inversion
        if len(self.depth) >= len(self.times):
            self.ax_d.plot(self.times, self.depth[:len(self.times)], 'c-', linewidth=1)
        else:
            self.ax_d.plot(self.times[:len(self.depth)], self.depth, 'c-', linewidth=1)

        self.ax_d.set_ylabel('Depth (m)')
        self.ax_d.set_title('Depth')
        self.ax_d.invert_yaxis()  # Invert after plotting and setting labels

        self.ax_h.plot(self.times, self.heading, 'g-', linewidth=1)
        self.ax_h.set_ylabel('Heading (rad)')
        self.ax_h.set_title('Heading (wrapped ±π)')
        self.ax_h.axhline(np.pi, linestyle='--', alpha=0.5)
        self.ax_h.axhline(-np.pi, linestyle='--', alpha=0.5)
        self.ax_h.set_ylim(-np.pi - 0.5, np.pi + 0.5)
        self.ax_h.set_xlabel('Elapsed Time (s)')

        # Remove grid and adjust layout
        for ax in (self.ax_n, self.ax_e, self.ax_d, self.ax_h):
            ax.set_facecolor('#f8f9fa')

        plt.tight_layout()

    def run(self):
        plt.ion()
        self.fig.show()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            plt.pause(0.1)
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = LivePosePlot()
    node.run()

if __name__ == '__main__':
    main()