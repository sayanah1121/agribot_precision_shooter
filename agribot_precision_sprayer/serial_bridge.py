#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import serial
import time

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        # Parameters (configurable via launch file)
        self.declare_parameter('port', '/dev/ttyACM0') 
        self.declare_parameter('baud', 115200)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.ser = None
        self.connect_serial(port, baud)

        # Subscribers
        self.sub_fire = self.create_subscription(
            Bool, 
            '/actuator/fire', 
            self.fire_callback, 
            10
        )
        
        self.sub_move = self.create_subscription(
            String, 
            '/cmd_vel_string', 
            self.move_callback, 
            10
        )

    def connect_serial(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2) # Wait for Arduino to reset
            self.get_logger().info(f"Successfully connected to Arduino on {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not connect to Arduino: {e}")
            self.get_logger().error("Check USB connection and permissions (sudo usermod -a -G dialout $USER)")

    def fire_callback(self, msg):
        if msg.data:
            self.send_command("FIRE")

    def move_callback(self, msg):
        # Expects strings like "FORWARD", "STOP", "LEFT"
        self.send_command(msg.data)

    def send_command(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                cmd_str = f"{cmd}\n"
                self.ser.write(cmd_str.encode('utf-8'))
                # self.get_logger().info(f"Sent: {cmd}")
            except Exception as e:
                self.get_logger().error(f"Serial Write Failed: {e}")
        else:
            self.get_logger().warn("Serial port not open, cannot send command")

    def __del__(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
