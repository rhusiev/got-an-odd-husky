import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.velocity_publisher = self.create_publisher(TwistStamped, "/husky/cmd_vel", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        twist_stamped_message = TwistStamped()
        twist_stamped_message.twist.linear.x = 2.0
        twist_stamped_message.twist.angular.z = 2.0

        self.velocity_publisher.publish(twist_stamped_message)
        self.get_logger().info('Publishing linear.x, angular.z: %f %f' % (2.0, 2.0))
        print(twist_stamped_message.twist)


def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
