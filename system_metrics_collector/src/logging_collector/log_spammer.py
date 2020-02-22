import rclpy
from rclpy.node import Node
import random


class LogSpammer(Node):

    def __init__(self):
        super().__init__('LogSpammer')
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    def timer_callback(self):

        r = random.random()

        if r < 0.90:
            self.get_logger().debug('Debug-ing')
        if r < 0.70:
            self.get_logger().info('Info-ing')
        if r < 0.30:
            self.get_logger().warn('Warn-ing')
        if r < 0.15:
            self.get_logger().error('Error-ing')
        if r < 0.05:
            self.get_logger().fatal('Fatal-ing')


def main(args=None):
    rclpy.init(args=args)
    log_spammer = LogSpammer()
    rclpy.spin(log_spammer)
    log_spammer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()