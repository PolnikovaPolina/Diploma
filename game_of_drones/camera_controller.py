import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class CameraController(Node):
    def __init__(self, node_name='camera_controller'):
        super().__init__(node_name)
        self.publisher = self.create_publisher(Float32MultiArray, '/camera_target', 10)

    def send_target_point(self, target_point):
        msg = Float32MultiArray()
        msg.data = target_point
        self.publisher.publish(msg)
        self.get_logger().info(f'Published camera target: {target_point}')