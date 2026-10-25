import os
import time

from rgbd_udp_receiver.image_preview import depth_preview_lut


class LocalImageViewer:
    """Display only the newest paired RGB-D frame in OpenCV windows."""

    def __init__(self, receiver):
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError(
                "Local image windows require opencv-python in the active "
                "Utonia environment (not opencv-python-headless)"
            ) from exc
        self._cv2 = cv2
        self._receiver = receiver
        self._period = 1.0 / receiver.config.gui_fps
        self._depth_lut = depth_preview_lut(receiver.config.max_depth_mm)

    def run(self, keep_running) -> None:
        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            raise RuntimeError("Local image windows require a desktop display")
        cv2 = self._cv2
        color_window = "RGB-D receiver: RGB"
        depth_window = "RGB-D receiver: depth (near=white)"
        next_display = time.monotonic()
        try:
            cv2.namedWindow(color_window, cv2.WINDOW_NORMAL)
            cv2.namedWindow(depth_window, cv2.WINDOW_NORMAL)
            while keep_running():
                now = time.monotonic()
                if now >= next_display:
                    bundle = self._receiver.take_latest_gui_bundle()
                    if bundle is not None:
                        started = time.perf_counter_ns()
                        color_bgr = cv2.cvtColor(
                            bundle.color.pixels, cv2.COLOR_RGB2BGR
                        )
                        depth_gray = self._depth_lut[bundle.depth.millimeters]
                        cv2.imshow(color_window, color_bgr)
                        cv2.imshow(depth_window, depth_gray)
                        self._receiver.record_gui_render(
                            (time.perf_counter_ns() - started) / 1_000_000.0
                        )
                    next_display = now + self._period
                remaining_ms = (next_display - time.monotonic()) * 1000.0
                wait_ms = min(10, max(1, int(remaining_ms)))
                key = cv2.waitKey(wait_ms) & 0xFF
                if key in (27, ord("q")):
                    return
        finally:
            cv2.destroyAllWindows()
