#!/usr/bin/env python3
"""Display the fields of the installed referee message once per second."""
import time

import rclpy
from rclpy.node import Node
from referee_msg.msg import Referee


class Subscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        self.last_print = float('-inf')
        self.subscriber = self.create_subscription(Referee, 'Referee', self.sub_callback, 10)

    def sub_callback(self, message):
        now = time.monotonic()
        if now - self.last_print < 1.0:
            return
        self.last_print = now
        print('\n'.join(f'[{field}]\t{getattr(message, field)}'
                        for field in message.get_fields_and_field_types()), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = Subscriber('referee_subscriber')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
