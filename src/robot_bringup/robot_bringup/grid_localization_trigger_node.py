#!/usr/bin/env python3

from typing import Optional

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Empty


class GridLocalizationTriggerNode(Node):
    """One-shot helper that triggers occupancy-grid localization with retries."""

    def __init__(self) -> None:
        super().__init__('grid_localization_trigger')

        self.declare_parameter('service_name', '/trigger_grid_search_localization')
        self.declare_parameter('startup_delay_sec', 2.5)
        self.declare_parameter('retries', 12)
        self.declare_parameter('retry_interval_sec', 1.0)
        self.declare_parameter('service_ready_timeout_sec', 90.0)
        self.declare_parameter('service_call_timeout_sec', 3.0)
        self.declare_parameter('result_topic', '/localization_result')

        self._service_name = str(self.get_parameter('service_name').value)
        self._startup_delay_sec = float(self.get_parameter('startup_delay_sec').value)
        self._retries = int(self.get_parameter('retries').value)
        self._retry_interval_sec = float(self.get_parameter('retry_interval_sec').value)
        self._service_ready_timeout_sec = float(
            self.get_parameter('service_ready_timeout_sec').value
        )
        self._service_call_timeout_sec = float(
            self.get_parameter('service_call_timeout_sec').value
        )
        self._result_topic = str(self.get_parameter('result_topic').value)

        if self._startup_delay_sec < 0.0:
            self._startup_delay_sec = 0.0
        if self._retries <= 0:
            self._retries = 1
        if self._retry_interval_sec <= 0.0:
            self._retry_interval_sec = 1.0
        if self._service_ready_timeout_sec <= 0.0:
            self._service_ready_timeout_sec = 90.0
        if self._service_call_timeout_sec <= 0.0:
            self._service_call_timeout_sec = 3.0

        self._client = self.create_client(Empty, self._service_name)
        self._attempt_count = 0
        self._pending_future = None
        self._pending_call_started_at = None
        self._retry_timer: Optional[object] = None
        self._service_wait_started_at = self.get_clock().now()
        self._last_service_wait_log_sec = -1.0
        self._result_received = False
        self._done = False

        self.create_subscription(
            PoseWithCovarianceStamped,
            self._result_topic,
            self._on_localization_result,
            10,
        )

        self._startup_timer = self.create_timer(
            self._startup_delay_sec,
            self._start_retry_loop,
        )

        self.get_logger().info(
            'Grid localization auto-trigger armed: '
            f'service={self._service_name}, startup_delay_sec={self._startup_delay_sec:.2f}, '
            f'retries={self._retries}, retry_interval_sec={self._retry_interval_sec:.2f}, '
            f'service_ready_timeout_sec={self._service_ready_timeout_sec:.2f}, '
            f'service_call_timeout_sec={self._service_call_timeout_sec:.2f}, '
            f'result_topic={self._result_topic}'
        )

    def _start_retry_loop(self) -> None:
        if self._startup_timer is not None:
            self._startup_timer.cancel()
            self._startup_timer = None

        self._retry_timer = self.create_timer(
            self._retry_interval_sec,
            self._attempt_trigger,
        )

        # Attempt immediately once delay expires.
        self._attempt_trigger()

    def _on_localization_result(self, _msg: PoseWithCovarianceStamped) -> None:
        if self._result_received:
            return
        self._result_received = True
        self.get_logger().info(
            f'Received localization result on {self._result_topic}; stopping auto-trigger.'
        )
        self._stop()

    def _attempt_trigger(self) -> None:
        if self._result_received:
            self._stop()
            return

        if self._pending_future is not None and not self._pending_future.done():
            if self._pending_call_started_at is not None:
                pending_elapsed_sec = (
                    self.get_clock().now().nanoseconds
                    - self._pending_call_started_at.nanoseconds
                ) / 1.0e9
                if pending_elapsed_sec >= self._service_call_timeout_sec:
                    self.get_logger().warning(
                        f'Grid localization trigger call timed out after '
                        f'{pending_elapsed_sec:.1f}s '
                        f'(attempt {self._attempt_count}/{self._retries}); retrying.'
                    )
                    self._pending_future.cancel()
                    self._pending_future = None
                    self._pending_call_started_at = None
                else:
                    return
            else:
                return

        if not self._client.wait_for_service(timeout_sec=0.1):
            elapsed_sec = (
                self.get_clock().now().nanoseconds - self._service_wait_started_at.nanoseconds
            ) / 1.0e9
            if elapsed_sec >= self._service_ready_timeout_sec:
                self.get_logger().error(
                    f'Grid localization trigger service did not become ready in '
                    f'{self._service_ready_timeout_sec:.1f}s: {self._service_name}'
                )
                self._stop()
                return
            if (self._last_service_wait_log_sec < 0.0) or (
                elapsed_sec - self._last_service_wait_log_sec >= 5.0
            ):
                self._last_service_wait_log_sec = elapsed_sec
                self.get_logger().warning(
                    f'Waiting for grid localization trigger service: {self._service_name} '
                    f'(elapsed {elapsed_sec:.1f}s/{self._service_ready_timeout_sec:.1f}s)'
                )
            return

        if self._attempt_count >= self._retries:
            self.get_logger().error(
                f'Failed to trigger grid localization via "{self._service_name}" '
                f'after {self._retries} service calls.'
            )
            self._stop()
            return

        self._attempt_count += 1
        attempt = self._attempt_count

        self.get_logger().info(
            f'Calling {self._service_name} (attempt {attempt}/{self._retries})'
        )
        self._pending_future = self._client.call_async(Empty.Request())
        self._pending_call_started_at = self.get_clock().now()
        self._pending_future.add_done_callback(self._on_trigger_done)

    def _on_trigger_done(self, future) -> None:
        if future.cancelled():
            self.get_logger().warning(
                f'Grid localization trigger call cancelled (attempt {self._attempt_count}/{self._retries})'
            )
            return

        try:
            future.result()
        except Exception as exc:  # pragma: no cover - runtime service error path
            self._pending_future = None
            self._pending_call_started_at = None
            self.get_logger().warning(
                f'Grid localization trigger failed on attempt '
                f'{self._attempt_count}/{self._retries}: {exc}'
            )
            return

        self._pending_future = None
        self._pending_call_started_at = None
        self.get_logger().info(
            f'Grid localization trigger call succeeded via {self._service_name}. '
            f'Waiting for localization result on {self._result_topic}.'
        )

    def _stop(self) -> None:
        if self._startup_timer is not None:
            self._startup_timer.cancel()
            self._startup_timer = None
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None
        self._done = True
        self.get_logger().info('Grid localization trigger node exiting.')

    @property
    def done(self) -> bool:
        return self._done


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = GridLocalizationTriggerNode()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
