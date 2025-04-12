import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthConverter(Node):
    def __init__(self):
        super().__init__('depth_converter')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/depth_camera',  # Subscribe to depth camera
            self.depth_callback,
            10
        )
        self.publisher = self.create_publisher(Image, '/depth_camera_mono8', 10)

    def depth_callback(self, msg):
        # Convert ROS Image message to OpenCV
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Normalize depth values (convert FLOAT32 to 8-bit grayscale)
        depth_image = np.nan_to_num(depth_image)  # Replace NaNs with 0
        depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_image = np.uint8(depth_image)

        # Convert back to ROS 2 Image and publish
        mono8_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="mono8")
        self.publisher.publish(mono8_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthConverter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
