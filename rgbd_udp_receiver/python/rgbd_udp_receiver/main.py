import rclpy

from .receiver_node import ReceiverNode


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ReceiverNode()
        node.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
