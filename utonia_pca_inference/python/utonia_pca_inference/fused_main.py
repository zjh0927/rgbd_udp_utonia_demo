import threading

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from rgbd_udp_receiver.receiver_node import ReceiverNode

from .inference_node import InferenceNode


def main(args=None):
    rclpy.init(args=args)
    receiver = None
    inference = None
    executor_thread = None
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
        viewer = None
        if receiver.config.gui_fps > 0:
            from .local_viewer import LocalImageViewer

            viewer = LocalImageViewer(receiver)
        inference.start()
        receiver.start()
        if viewer is None:
            executor.spin()
        else:

            def spin_ros():
                try:
                    executor.spin()
                except ExternalShutdownException:
                    pass

            executor_thread = threading.Thread(
                target=spin_ros, name="rgbd-ros-executor", daemon=True
            )
            executor_thread.start()
            viewer.run(lambda: rclpy.ok() and executor_thread.is_alive())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if receiver is not None:
            receiver.close()
        if inference is not None:
            inference.close()
        executor.shutdown()
        if executor_thread is not None:
            executor_thread.join(timeout=5.0)
        if receiver is not None:
            receiver.destroy_node()
        if inference is not None:
            inference.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
