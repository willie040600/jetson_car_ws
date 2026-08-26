#!/usr/bin/env python3
"""
ZLAC706-RC 差速輪 ROS2 節點 (rclpy)
------------------------------------------------
底層 Modbus RTU 通訊採用手動組封包 + CRC16（不依賴 pymodbus），
協定與暫存器對應到已驗證可動的測試腳本：
  - 左輪 slave address = 0x01, 右輪 slave address = 0x02
  - 寫入速度命令：暫存器 0x0011 (function code 0x06)
  - 讀取狀態+速度：暫存器 0x00D2, 長度 2 (function code 0x03)
  - 使能指令：對 0x0010 寫入固定值（每輪各自的 CRC 已內嵌）
  - RPM <-> 內部整數換算：data = rpm / 3000 * 8192

安裝需求 (在 Jetson Nano 上執行)：
  pip3 install pyserial --break-system-packages
  sudo usermod -a -G dialout $USER   # 需要重新登入才生效

功能：
  - 訂閱 /cmd_vel (geometry_msgs/Twist)，換算成左右輪目標 RPM，寫入驅動板
  - 定時讀取左右輪實際 RPM，推算里程計，發布 /odom 與 TF (odom -> base_link)
  - 逾時未收到新的 /cmd_vel 會自動歸零（安全機制）
"""

import time
import math

import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


# ---------- Modbus CRC ----------
def modbus_crc(arr: bytes) -> int:
    crc = 0xFFFF
    for b in arr:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(payload: bytearray) -> bytes:
    crc = modbus_crc(payload)
    payload += bytes([crc & 0xFF, (crc >> 8) & 0xFF])  # little-endian CRC
    return bytes(payload)


def build_speed_cmd(addr: int, rpm: float) -> bytes:
    """寫單一保持暫存器 0x0011：速度命令 (data = rpm/3000 * 8192)"""
    data = int(rpm / 3000.0 * 8192.0)
    data &= 0xFFFF  # 兩補數
    payload = bytearray([addr, 0x06, 0x00, 0x11, (data >> 8) & 0xFF, data & 0xFF])
    return append_crc(payload)


def read_strpm_cmd(addr: int) -> bytes:
    """讀保持暫存器 0x00D2, 長度 2（狀態 + 速度）"""
    payload = bytearray([addr, 0x03, 0x00, 0xD2, 0x00, 0x02])
    return append_crc(payload)


def parse_speed_reply(data: bytes, expect_addr: int):
    """回傳 (state:int, speed_raw:int16)；格式不對回傳 (None, None)"""
    if len(data) >= 9 and data[0] == expect_addr and data[1] == 0x03 and data[2] == 0x04:
        state = (data[3] << 8) | data[4]
        raw = (data[5] << 8) | data[6]
        if raw >= 0x8000:
            raw -= 0x10000
        return state, raw
    return None, None


def rpm_from_raw(raw: int) -> float:
    return raw * 3000.0 / 8192.0


# 使能命令（沿用已驗證可動的固定 bytes，內含各自的 CRC）
ENABLE_L = bytes([0x01, 0x06, 0x00, 0x10, 0x00, 0x1F, 0xC9, 0xC7])
ENABLE_R = bytes([0x02, 0x06, 0x00, 0x10, 0x00, 0x1F, 0xC9, 0xF4])

ADDR_L = 0x01
ADDR_R = 0x02

# 0x06 的回應是原封包回音 8 bytes；0x03 讀 2 個暫存器是 9 bytes
WRITE_REPLY_LEN = 8
READ_REPLY_LEN = 9
REPLY_TIMEOUT = 0.03
# Modbus RTU 幀間靜默：38400bps 下 3.5 字元約 0.9ms，取 2ms 保守值
SILENT_INTERVAL = 0.002


class ZLAC706DiffDrive(Node):
    def __init__(self):
        super().__init__('zlac706_diffdrive_node')

        # ---- 參數，依實際機構調整 ----
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 38400)
        self.declare_parameter('wheel_radius', 0.0508)   # 公尺
        self.declare_parameter('wheel_separation', 0.23) # 公尺，左右輪中心距
        self.declare_parameter('max_rpm', 200.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', True)  # 左右輪馬達通常反向安裝
        self.declare_parameter('cmd_timeout', 0.5)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.invert_left = self.get_parameter('invert_left').value
        self.invert_right = self.get_parameter('invert_right').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        # ---- 序列埠連線 ----
        try:
            self.ser = serial.Serial(port, baud, timeout=REPLY_TIMEOUT)
            self.get_logger().info(f'序列埠已開啟: {port} @ {baud}bps')
            self._enable_driver()
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'無法開啟序列埠 {port}: {e}')

        # ---- ROS interfaces ----
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---- 里程計狀態 ----
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # 序列埠只由 control_cb 這一條路徑存取，cmd_vel 僅更新目標值，避免搶匯流排
        self.target_rpm_l = 0.0
        self.target_rpm_r = 0.0
        self.last_cmd_time = time.time()

        period = 1.0 / self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(period, self.control_cb)

    # ----------------------------------------------------------------
    def _transact(self, request: bytes, expect_len: int) -> bytes:
        """送出一筆 Modbus RTU 請求並收完整個回應，結束後保留幀間靜默。"""
        self.ser.reset_input_buffer()
        self.ser.write(request)
        self.ser.flush()
        data = self.ser.read(expect_len)
        time.sleep(SILENT_INTERVAL)
        return data

    def _enable_driver(self):
        if self.ser is None:
            return
        self._transact(ENABLE_L, WRITE_REPLY_LEN)
        self._transact(ENABLE_R, WRITE_REPLY_LEN)

    def _write_wheel_rpm(self, left_rpm: float, right_rpm: float):
        if self.ser is None:
            return
        left_rpm = max(-self.max_rpm, min(self.max_rpm, left_rpm))
        right_rpm = max(-self.max_rpm, min(self.max_rpm, right_rpm))

        if self.invert_left:
            left_rpm = -left_rpm
        if self.invert_right:
            right_rpm = -right_rpm

        try:
            self._transact(build_speed_cmd(ADDR_L, left_rpm), WRITE_REPLY_LEN)
            self._transact(build_speed_cmd(ADDR_R, right_rpm), WRITE_REPLY_LEN)
        except Exception as e:
            self.get_logger().warn(f'寫入速度命令失敗: {e}')

    def _read_wheel_rpm(self):
        if self.ser is None:
            return None
        try:
            data_l = self._transact(read_strpm_cmd(ADDR_L), READ_REPLY_LEN)
            data_r = self._transact(read_strpm_cmd(ADDR_R), READ_REPLY_LEN)

            _, raw_l = parse_speed_reply(data_l, ADDR_L)
            _, raw_r = parse_speed_reply(data_r, ADDR_R)
            if raw_r is None or raw_l is None:
                return None

            rpm_l = rpm_from_raw(raw_l)
            rpm_r = rpm_from_raw(raw_r)
            if self.invert_left:
                rpm_l = -rpm_l
            if self.invert_right:
                rpm_r = -rpm_r
            return rpm_l, rpm_r
        except Exception as e:
            self.get_logger().warn(f'讀取轉速回授失敗: {e}')
            return None

    # ----------------------------------------------------------------
    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd_time = time.time()
        # 實測發現前後方向與 /cmd_vel 定義相反，這裡整體翻轉線速度；
        # 不動 w，也不動 invert_left/invert_right，避免動到已經調對的轉向。
        v = -msg.linear.x      # m/s
        w = msg.angular.z      # rad/s

        v_left = v - (w * self.wheel_sep / 2.0)
        v_right = v + (w * self.wheel_sep / 2.0)

        self.target_rpm_l = (v_left / (2 * math.pi * self.wheel_radius)) * 60.0
        self.target_rpm_r = (v_right / (2 * math.pi * self.wheel_radius)) * 60.0

    # ----------------------------------------------------------------
    def control_cb(self):
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            self.target_rpm_l = 0.0
            self.target_rpm_r = 0.0

        self._write_wheel_rpm(self.target_rpm_l, self.target_rpm_r)
        self._update_odom(self._read_wheel_rpm())

    def _update_odom(self, fb):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if fb is None or dt <= 0.0:
            return

        rpm_left, rpm_right = fb
        v_left = (rpm_left / 60.0) * 2 * math.pi * self.wheel_radius
        v_right = (rpm_right / 60.0) * 2 * math.pi * self.wheel_radius

        v = (v_left + v_right) / 2.0
        w = (v_right - v_left) / self.wheel_sep

        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = w * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        quat_z = math.sin(self.theta / 2.0)
        quat_w = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = quat_z
        odom.pose.pose.orientation.w = quat_w
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = quat_z
        t.transform.rotation.w = quat_w
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        try:
            self._write_wheel_rpm(0.0, 0.0)
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZLAC706DiffDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
