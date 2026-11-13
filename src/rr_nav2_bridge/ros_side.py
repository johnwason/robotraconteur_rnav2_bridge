"""ROS 2 side of the bridge: one rclpy node spun by a MultiThreadedExecutor on a
background thread. RR handler threads never spin rclpy; they call the thread-safe
publish helpers here or wait on rclpy futures with timeouts."""
from __future__ import annotations

import logging
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

log = logging.getLogger("rr_nav2_bridge.ros")


class RosSide:
    def __init__(self, node_name: str = "rr_nav2_bridge"):
        rclpy.init(args=None)
        self.node = Node(node_name)
        self.cb = ReentrantCallbackGroup()
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, name="rclpy-executor", daemon=True)

    def start(self):
        self._thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:  # pragma: no cover - logged, process supervisor restarts
            log.exception("rclpy executor stopped")

    def now_msg(self):
        return self.node.get_clock().now().to_msg()

    @staticmethod
    def wall() -> float:
        """Service timestamp (UNIX seconds) reported to RR clients."""
        return time.time()

    def wait_future(self, fut, timeout: float):
        """Block (on an RR thread) until an rclpy future completes. Returns the result or
        raises TimeoutError."""
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout):
            raise TimeoutError()
        return fut.result()

    def shutdown(self):
        try:
            self.executor.shutdown(timeout_sec=2.0)
        finally:
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
