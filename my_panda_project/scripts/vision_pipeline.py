#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import re

class TargetCoordinator(Node):
    def __init__(self):
        super().__init__('target_coordinator')
        # Subscriptions and Publishers
        self.image_sub = self.create_subscription(Image, '/workspace_camera/image_raw', self.image_callback, 10)
        self.voice_sub = self.create_subscription(String, '/voice_commands', self.voice_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.color_pub = self.create_publisher(String, '/target_color', 10)
        self.bridge = CvBridge()
        
        self.active_color = None
        self.get_logger().info("Target Coordinator Active. Awaiting voice commands...")

    def voice_callback(self, msg):
        text = msg.data.lower()
        
        # Regex: Added 'pick' so it catches both 'pick up' and just 'pick'
        match = re.search(r'(pick up|pick|grab|target|get).*?(red|green|blue)', text)
        
        if match:
            # Extract the actual color attached to the command (group 2 in the regex)
            self.active_color = match.group(2).capitalize()
            self.get_logger().info(f"Intent recognized. Target locked: {self.active_color} block.")
        else:
            # If they just say "ignore red" without a valid pick up command, do nothing
            self.get_logger().info(f"No valid action intent found in: '{text}'")

    def pixel_to_3d(self, cx, cy):
        # 1. Camera Intrinsics
        W, H, FOV = 640, 480, 1.3962634
        f = (W / 2) / math.tan(FOV / 2)
        
        # 2. Known physical heights
        Z_cam = 1.50    # The exact Z height set in the SDF
        Z_target = 0.45 # The table height
        block_height = 0.025   # The 5cm block
        Z_table = 0.45
        
        # Target the TOP of the block, not the table surface
        Z_target = Z_table + block_height
        distance = Z_cam - Z_target
        
        # 3. Known camera center in the world
        X_cam = 0.80    # The exact X position set in the SDF
        Y_cam = 0.00    # The exact Y position set in the SDF
        
        # 4. Direct 2D-to-3D projection
        # Since the camera looks straight down:
        X_w = X_cam + ((H / 2 - cy) * distance) / f
        Y_w = Y_cam + ((W / 2 - cx) * distance) / f
        
        # 5. Let ROS 2 TF handle the offsets! We return pure World Coordinates.
        return float(X_w), float(Y_w), float(Z_target)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Idle state if no command received
        if not self.active_color:
            cv2.putText(frame, "Say a color command...", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow("3D Target Coordinator", frame)
            cv2.waitKey(1)
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = None
        
        # Apply specific mask based on voice command
        if self.active_color == 'Red':
            mask1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
            mask2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
            mask = cv2.bitwise_or(mask1, mask2)
            box_color = (0, 0, 255)
        elif self.active_color == 'Green':
            mask = cv2.inRange(hsv, np.array([40, 100, 100]), np.array([80, 255, 255]))
            box_color = (0, 255, 0)
        elif self.active_color == 'Blue':
            mask = cv2.inRange(hsv, np.array([100, 100, 100]), np.array([140, 255, 255]))
            box_color = (255, 0, 0)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) > 100:
                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w / 2
                cy = y + h / 2
                
                # Compute Absolute 3D World Coordinates
                x_world, y_world, z_world = self.pixel_to_3d(cx, cy)
                
                # Publish Pose to ROS 2 using the absolute 'world' frame
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'world'
                pose_msg.pose.position.x = x_world
                pose_msg.pose.position.y = y_world
                pose_msg.pose.position.z = z_world
                self.pose_pub.publish(pose_msg)
                color_msg = String()
                color_msg.data = self.active_color
                self.color_pub.publish(color_msg)
                
                # RESET THE STATE so it doesn't flood MoveIt with continuous requests
                self.active_color = None
                
                # Draw annotations
                cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
                cv2.circle(frame, (int(cx), int(cy)), 3, (255, 255, 255), -1)
                
                # Display the Absolute World coordinates on screen
                label = f"Target {self.active_color}: X:{x_world:.2f} Y:{y_world:.2f} Z:{z_world:.2f}"
                cv2.putText(frame, label, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)
                break 
        
        cv2.imshow("3D Target Coordinator", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TargetCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()