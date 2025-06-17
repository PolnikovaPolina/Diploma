import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from my_interface.msg import CameraStamped
from std_msgs.msg import Float32MultiArray

class Observer(Node):
    def __init__(self, mediator, node_name='observer'):
        super().__init__(node_name)

        qos_profile_unity = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )

        self.mediator = mediator

        self.real_pose_sub = self.create_subscription(
            Float32MultiArray,
            '/object_pixel_pose',
            self.real_pose_callback,
            qos_profile_unity)

        self.cv_sub = self.create_subscription(
            CameraStamped,
            '/camera_image_status',
            self.camera_image_callback,
            qos_profile_unity)

    def camera_image_callback(self, msg):
        self.mediator.handle_camera_image(msg)

    def real_pose_callback(self, msg):
        self.mediator.add_real(msg.data)

# Відслідковувати та отримувати повідомлення з ROS-топіків.
# Передавати отримані дані в Mediator.
# клас успадковує Node, бо безпосередньо взаємодіє з ROS2 API.