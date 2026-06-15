#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    import serial
    from serial.serialutil import SerialException
except ImportError:
    serial = None
    SerialException = Exception


class ArduinoMetalSensor(Node):
    def __init__(self):
        super().__init__('arduino_metal_sensor')

        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('sensor_state_topic', '/metal_sensor/any_detected')
        self.declare_parameter('command_topic', '/arduino/command')
        self.declare_parameter('event_topic', '/arduino/events')
        self.declare_parameter('raw_topic', '/arduino/raw')
        self.declare_parameter('read_timeout_sec', 0.1)
        self.declare_parameter('publish_raw', False)

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.sensor_state_topic = self.get_parameter('sensor_state_topic').value
        self.command_topic = self.get_parameter('command_topic').value
        self.event_topic = self.get_parameter('event_topic').value
        self.raw_topic = self.get_parameter('raw_topic').value
        self.publish_raw = bool(self.get_parameter('publish_raw').value)

        self.any_detected_pub = self.create_publisher(Bool, self.sensor_state_topic, 10)
        self.event_pub = self.create_publisher(String, self.event_topic, 10)
        self.raw_pub = self.create_publisher(String, self.raw_topic, 10)
        self.command_sub = self.create_subscription(String, self.command_topic, self.on_command, 10)

        self.serial_port_handle = None
        self.buffer = b''
        self.open_attempt_time = self.get_clock().now()

        self.create_timer(0.05, self.read_serial)

        if serial is None:
            self.get_logger().fatal('pyserial is not installed. Install it with pip install pyserial.')

    def open_serial_connection(self):
        if serial is None:
            return False
        if self.serial_port_handle is not None and self.serial_port_handle.is_open:
            return True
        now = self.get_clock().now()
        if (now - self.open_attempt_time).nanoseconds < 2_000_000_000:
            return False
        self.open_attempt_time = now
        try:
            self.serial_port_handle = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=self.get_parameter('read_timeout_sec').value,
            )
            self.get_logger().info(f'Opened Arduino serial port: {self.serial_port} @ {self.baudrate}')
            return True
        except SerialException as exc:
            self.get_logger().warn(f'Unable to open Arduino serial port {self.serial_port}: {exc}')
            self.serial_port_handle = None
            return False

    def read_serial(self):
        if not self.open_serial_connection():
            return

        try:
            while self.serial_port_handle.in_waiting > 0:
                data = self.serial_port_handle.read(self.serial_port_handle.in_waiting)
                if not data:
                    break
                self.buffer += data
                while b'\n' in self.buffer:
                    line, self.buffer = self.buffer.split(b'\n', 1)
                    self.process_line(line.decode('utf-8', errors='ignore').strip())
        except SerialException as exc:
            self.get_logger().warn(f'Arduino serial read failed: {exc}')
            self.serial_port_handle = None

    def process_line(self, line: str):
        if not line:
            return

        self.event_pub.publish(String(data=line))

        if self.publish_raw:
            self.raw_pub.publish(String(data=line))

        if line.startswith('METAL,'):
            tokens = line.split(',')
            if len(tokens) == 5:
                any_detected = tokens[4].strip() == '1'
                self.any_detected_pub.publish(Bool(data=any_detected))

    def on_command(self, msg: String):
        if not self.open_serial_connection():
            self.get_logger().warn('Cannot send Arduino command: serial port not open.')
            return

        try:
            command = msg.data.strip()
            if not command:
                return
            self.serial_port_handle.write((command + '\n').encode('utf-8'))
            self.get_logger().info(f'Sent Arduino command: {command}')
        except SerialException as exc:
            self.get_logger().warn(f'Failed to send Arduino command: {exc}')
            self.serial_port_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoMetalSensor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
