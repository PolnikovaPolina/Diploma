import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import CompressedImage  # Import the necessary message type
# from geometry_msgs.msg import PoseStamped

import cv2
import numpy as np

class CameraView(Node):
    def __init__(self):
        super().__init__('roscon_module')
        qos_profile_unity = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        # Create subscribers
        self.camera_image_subscriber = self.create_subscription(
            CompressedImage,
            '/camera_image',
            self.camera_image_callback,
            qos_profile_unity
        )

    def camera_image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


        cv2.imshow('Camera Image', image_np)
          # Compute the gradient image using Sobel operator
        gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
        grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        gradient = cv2.magnitude(grad_x, grad_y)
        gradient = cv2.convertScaleAbs(gradient)
        
        # Display the gradient image
        cv2.imshow('Gradient Image', gradient)
        
        cv2.waitKey(1) 


def main(args=None):
    rclpy.init(args=args)
    node = CameraView()
    rclpy.spin(node)

    # Destroy the node after the program is stopped
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
