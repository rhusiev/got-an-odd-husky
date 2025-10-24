import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import numpy as np
from scipy.ndimage import convolve1d
import threading


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        self.sensor_callback_group = MutuallyExclusiveCallbackGroup()
        self.control_callback_group = MutuallyExclusiveCallbackGroup()

        self.velocity_publisher = self.create_publisher(
            TwistStamped, "/husky/cmd_vel", 10
        )

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            "/husky/sensors/lidar2d_0/scan",
            self.lidar_callback,
            10,
            callback_group=self.sensor_callback_group,
        )

        self.control_timer = self.create_timer(
            0.1, self.control_loop, callback_group=self.control_callback_group
        )

        self.latest_scan = None
        self.last_scan_time = None
        self.data_lock = threading.Lock()

        self.get_logger().info("Controller initialized with separate callback groups")

    def lidar_callback(self, msg):
        with self.data_lock:
            self.latest_scan = msg
            self.last_scan_time = self.get_clock().now()
        self.get_logger().info("Lidar data received", throttle_duration_sec=1.0)

    def control_loop(self):
        with self.data_lock:
            scan = self.latest_scan
            scan_time = self.last_scan_time

        twist = TwistStamped()

        if scan is None:
            self.get_logger().warn("No lidar data yet", throttle_duration_sec=1.0)
            return

        scan_age = (self.get_clock().now() - scan_time).nanoseconds / 1e9
        if scan_age > 0.5:
            self.get_logger().error(
                "Lidar data stale, stopping", throttle_duration_sec=1.0
            )
            twist.twist.linear.x = 0.0
            twist.twist.angular.z = 0.0
            self.velocity_publisher.publish(twist)
            return

        twist.twist.linear.x = 0.0
        twist.twist.angular.z = 0.4 # left

        self.velocity_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
