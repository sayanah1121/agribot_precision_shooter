#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point

class WeedDetector(Node):
    def __init__(self):
        super().__init__('weed_detector')
        
        # Open default camera (Index 0). Change to 1 if using USB cam on some setups.
        self.cap = cv2.VideoCapture(0)
        
        # Publishers
        self.publisher_ = self.create_publisher(Point, '/weed/pixel_coords', 10)
        self.image_pub_ = self.create_publisher(Image, '/camera/debug_image', 10)
        
        self.br = CvBridge()
        
        # Timer for 10 FPS
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Pi5 Vision System Started")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Could not read frame from camera")
            return
        
        # 1. Resize for performance on Pi
        frame = cv2.resize(frame, (320, 240))
        
        # 2. Green Masking (HSV)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Adjust these values based on your lighting and weed color
        lower_green = np.array([35, 40, 40])
        upper_green = np.array([85, 255, 255])
        
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # 3. Find Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        weed_found = False
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Filter noise (small blobs)
            if area > 300: 
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # Publish Coordinates
                    msg = Point()
                    msg.x = float(cx)
                    msg.y = float(cy)
                    self.publisher_.publish(msg)
                    
                    # Visual Debugging (Draw circle)
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    weed_found = True

        # Publish Debug Image for Rviz or rqt_image_view
        if weed_found:
             self.image_pub_.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = WeedDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
