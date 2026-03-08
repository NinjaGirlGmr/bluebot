#!/usr/bin/env python3

from copy import deepcopy
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def _zero_imu_message(frame_id: str = 'imu_link') -> Imu:
  msg = Imu()
  msg.orientation.w = 1.0
  msg.angular_velocity_covariance[0] = -1.0
  msg.linear_acceleration_covariance[0] = -1.0
  msg.orientation_covariance[0] = 0.0
  msg.orientation_covariance[4] = 0.0
  msg.orientation_covariance[8] = 0.0
  msg.header.frame_id = frame_id
  return msg


class ImuNode(Node):
  def __init__(self) -> None:
    super().__init__('imu_node')

    self.declare_parameter('source_topics', ['/camera/imu', '/camera/imu/data', '/imu/data', '/imu'])
    self.declare_parameter('publish_rate', 50.0)
    self.declare_parameter('raw_topic', '/imu/data_raw')
    self.declare_parameter('orientation_topic', '/imu/orientation')
    self.declare_parameter('frame_id', 'imu_link')
    self.declare_parameter('fallback_publish', True)

    source_topics = self.get_parameter('source_topics').get_parameter_value().string_array_value
    self._source_topics = list(source_topics) if source_topics else []
    publish_rate = float(self.get_parameter('publish_rate').get_parameter_value().double_value)
    self._raw_topic = self.get_parameter('raw_topic').get_parameter_value().string_value
    self._orientation_topic = self.get_parameter('orientation_topic').get_parameter_value().string_value
    self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
    self._fallback_publish = bool(self.get_parameter('fallback_publish').value)

    if publish_rate <= 0.0:
      publish_rate = 50.0
    if not self._source_topics:
      self._source_topics = ['/camera/imu', '/camera/imu/data', '/imu/data', '/imu']

    self._latest: Optional[Imu] = None
    self._last_source_received = False
    self._missing_source_warning_emitted = False

    self._raw_pub = self.create_publisher(Imu, self._raw_topic, 10)
    self._orientation_pub = self.create_publisher(Imu, self._orientation_topic, 10)

    for topic in self._source_topics:
      self.create_subscription(Imu, topic, self._on_imu, 10)
      self.get_logger().info(f'Subscribing to IMU input topic: {topic}')

    timer_period = 1.0 / publish_rate
    self.create_timer(timer_period, self._on_timer)
    self.get_logger().info(
      f'IMU bridge started. source_topics={self._source_topics}, '
      f'raw_topic={self._raw_topic}, orientation_topic={self._orientation_topic}, '
      f'rate={publish_rate:.1f} Hz'
    )

  def _on_imu(self, msg: Imu) -> None:
    self._latest = msg
    self._last_source_received = True
    self._publish_messages(msg)

  def _orientation_only(self, raw_msg: Imu) -> Imu:
    msg = Imu()
    msg.header = deepcopy(raw_msg.header)
    msg.orientation = deepcopy(raw_msg.orientation)
    msg.orientation_covariance = deepcopy(raw_msg.orientation_covariance)
    for i in range(9):
      msg.angular_velocity_covariance[i] = -1.0
      msg.linear_acceleration_covariance[i] = -1.0
    msg.angular_velocity.x = 0.0
    msg.angular_velocity.y = 0.0
    msg.angular_velocity.z = 0.0
    msg.linear_acceleration.x = 0.0
    msg.linear_acceleration.y = 0.0
    msg.linear_acceleration.z = 0.0
    msg.header.frame_id = self._frame_id
    return msg

  def _publish_messages(self, source: Imu) -> None:
    raw_msg = deepcopy(source)
    raw_msg.header.frame_id = self._frame_id if raw_msg.header.frame_id == '' else raw_msg.header.frame_id
    ori_msg = self._orientation_only(source)
    self._raw_pub.publish(raw_msg)
    self._orientation_pub.publish(ori_msg)

  def _on_timer(self) -> None:
    if self._latest is not None:
      self._publish_messages(self._latest)
      return
    if not self._fallback_publish:
      return
    self._raw_pub.publish(_zero_imu_message(self._frame_id))
    self._orientation_pub.publish(_zero_imu_message(self._frame_id))
    if not self._last_source_received and not self._missing_source_warning_emitted:
      self.get_logger().warn('No IMU source yet; publishing zeroed IMU placeholders.')
      self._missing_source_warning_emitted = True


def main(args=None) -> None:
  rclpy.init(args=args)
  node = ImuNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
