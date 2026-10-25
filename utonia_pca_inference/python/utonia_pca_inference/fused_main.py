import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from rgbd_udp_receiver.receiver_node import ReceiverNode

from .inference_node import InferenceNode


def main(args=None):
    rclpy.init(args=args)
    receiver = None
    inference = None
    executor = SingleThreadedExecutor()
    try:
        inference = InferenceNode(direct_input=True)
        receiver = ReceiverNode(point_callback=inference.submit_points)
        if receiver.config.max_output_points > inference.config.max_model_points:
            receiver.get_logger().warning(
                "Receiver projects %d points, but Utonia accepts at most %d; "
                "set max_output_points to %d to avoid extra work"
                % (
                    receiver.config.max_output_points,
                    inference.config.max_model_points,
                    inference.config.max_model_points,
                )
            )
        executor.add_node(receiver)
        executor.add_node(inference)
        inference.start()
        receiver.start()
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if receiver is not None:
            receiver.close()
        if inference is not None:
            inference.close()
        executor.shutdown()
        if receiver is not None:
            receiver.destroy_node()
        if inference is not None:
            inference.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
