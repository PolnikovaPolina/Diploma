import rclpy
from rclpy.node import Node
from my_interface.msg import CameraStamped
from GameOfDronesDev.game_of_drones.camera_controller import CameraController
from GameOfDronesDev.game_of_drones.detection.computer_vision import ComputerVision
from GameOfDronesDev.game_of_drones.detection.object_detection import SobelDetection
from GameOfDronesDev.game_of_drones.detection.contour_detection import LargestContour
from GameOfDronesDev.game_of_drones.visualization_analysis.analysis_results import AnalysisResults

class TestTrack(Node):
    def __init__(self, shared_dir = None, name_file = None):
        super().__init__('camera_test_node')

        self.shared_dir = shared_dir
        self.name_file = name_file

        self.cv_processor = ComputerVision(detection_strategy=SobelDetection(), contour_strategy=LargestContour())
        self.analysis_results = AnalysisResults(shared_dir = self.shared_dir, name_file=self.name_file)
        self.controller = CameraController(track_target = False, hit_target = False, analysis_results=self.analysis_results)

        self.subscription = self.create_subscription(
            CameraStamped,
            '/camera_image_status',
            self.callback,
            10
        )

    def callback(self, msg):
        measurement = self.cv_processor.get_measurement_frame(msg)
        self.controller.cv_processor = self.cv_processor  # на всяк випадок
        self.controller.track_drone(measurement)

def main(args=None):
    SHARED_DIR = '/media/sf_data'
    NAME_FILE = 'test_01'
    rclpy.init(args=args)
    node = TestTrack(shared_dir=SHARED_DIR, name_file=NAME_FILE)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Завершено вручну.")
    finally:
        print("Зберігаємо всі графіки...")
        node.analysis_results.save_all()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
