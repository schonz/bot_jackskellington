import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BotController(Node):
    def __init__(self):
        super().__init__('BotController')
        self.publisher = self.create_publisher(
            String,
            'servo_commands',
            10
        )

        self.ch0Pos = 0
        self.ch1Pos = 0
        timer_period = 0.5  # sec
        # -- temp --
        self.ch0Dir = 1
        self.ch1Dir = 1

        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        if self.ch0Pos >= 100:
            self.ch0Dir = -1
        elif self.ch0pos <= 0:
            self.ch0Dir = 1

        self.ch0Pos += self.ch0Dir

        msg = String()
        msg.data = '0:%d' % self.ch0Pos
        self.publisher.publish(msg)
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)

    botController = BotController()

    rclpy.spin(botController)

    # == Clean up ==
    botController.destroy_node()
    rclpy.shutdown()


if __main__ == '__main__':
    main()
