def main(args=None):
    import rclpy

    from .inference_node import InferenceNode

    rclpy.init(args=args)
    node = None
    try:
        node = InferenceNode()
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
