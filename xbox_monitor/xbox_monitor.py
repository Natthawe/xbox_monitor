#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool

class XboxMonitor(Node):
    def __init__(self):
        super().__init__('xbox_monitor')

        # ===== Parameters =====
        self.declare_parameter('device_path', '/dev/input/js_xbox_bt')
        self.declare_parameter('pub_topic', '/joystick_connected')
        self.declare_parameter('check_rate_hz', 5.0)          # เช็คทุก 200 ms = 5 Hz
        self.declare_parameter('heartbeat_sec', 0.0)          # 0 = ส่งเฉพาะตอนสถานะเปลี่ยน
        self.declare_parameter('publish_on_change_only', True)

        self._device_path = self.get_parameter('device_path').get_parameter_value().string_value
        self._pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self._check_rate_hz = float(self.get_parameter('check_rate_hz').value)
        self._heartbeat_sec = float(self.get_parameter('heartbeat_sec').value)
        self._change_only = bool(self.get_parameter('publish_on_change_only').value)

        # QoS: ให้ subscriber ใหม่เห็นค่าล่าสุดทันที
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(Bool, self._pub_topic, qos)

        # สถานะล่าสุดที่ประกาศไปแล้ว
        # state: True=connected, False=disconnected, None=ยังไม่เคยประกาศ
        self._last_state = None

        # ตั้ง timer เช็คไฟล์อุปกรณ์
        period = 1.0 / self._check_rate_hz if self._check_rate_hz > 0 else 0.2
        self._check_timer = self.create_timer(period, self._check_once)

        # Optional heartbeat to republish current state periodically (seconds)
        self._hb_timer = None
        if self._heartbeat_sec > 0.0:
            self._hb_timer = self.create_timer(self._heartbeat_sec, self._heartbeat)

        # เริ่มต้นประกาศสถานะครั้งแรก
        self._check_once()
        self.get_logger().info(
            f'Monitoring device="{self._device_path}", publish="{self._pub_topic}", '
            f'rate={self._check_rate_hz} Hz, heartbeat={self._heartbeat_sec}s, change_only={self._change_only}'
        )

    def _device_connected(self) -> bool:
        # เช็คว่ามีไฟล์อุปกรณ์ (หรือ symlink) อยู่ไหม
        return os.path.exists(self._device_path)

    def _publish(self, state: bool, force: bool = False):
        if self._change_only and not force and self._last_state is not None and state == self._last_state:
            return
        self._last_state = state
        self._pub.publish(Bool(data=state))
        # log เฉพาะตอนเปลี่ยนเพื่อลดสแปม
        # self.get_logger().info(f'joystick_connected = {state}')

    def _check_once(self):
        state = self._device_connected()
        self._publish(state)

    def _heartbeat(self):
        # ส่งค่าปัจจุบันซ้ำ ๆ แม้สถานะไม่เปลี่ยน
        cur = self._device_connected()
        self._publish(cur, force=True)

def main():
    rclpy.init()
    node = XboxMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
