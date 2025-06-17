import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from my_interface.msg import CameraTargetStamped
import numpy as np
import cv2

class Test(Node):
    def __init__(self):
        super().__init__('camera_target_listener')

        self.subscription = self.create_subscription(
            CameraTargetStamped,
            '/camera_target_data',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Зображення
        np_arr = np.frombuffer(msg.image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
        grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        gradient = cv2.magnitude(grad_x, grad_y)
        gradient = cv2.convertScaleAbs(gradient)

        # Дані позицій та FOV
        cam = msg.camera_pose.pose.position
        tgt = msg.target_pose.pose.position
        fov_data = msg.data.data  # Float32MultiArray

        print(f"[INFO] Camera position: x={cam.x:.2f}, y={cam.y:.2f}, z={cam.z:.2f}")
        print(f"[INFO] Target position: x={tgt.x:.2f}, y={tgt.y:.2f}, z={tgt.z:.2f}")
        print(f"[INFO] FOV: {fov_data}")

        # Display the gradient image
        cv2.imshow('Gradient Image', gradient)

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Test()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
