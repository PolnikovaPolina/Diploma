import rclpy, cv2
# from rclpy.executors import SingleThreadedExecutor

from observer import Observer
from mediator import Mediator
from computer_vision import ComputerVision
from object_detection import SobelDetection
from contour_detection import LargestContour
from unscented_filter_kalman import UnscentedFilterKalman
from analysis_results import AnalysisResults
from motion_model import FullMotionModel, SimpleMotionModel

def main(args=None):

    SHARED_DIR = '/media/sf_data'
    rclpy.init(args=args)

    # Створення екземплярів класів
    cv_processor = ComputerVision(detection_strategy=SobelDetection(), contour_strategy=LargestContour())

    #kalman_filter = UnscentedFilterKalman(FullMotionModel())
    kalman_filter = UnscentedFilterKalman(SimpleMotionModel())

    # Створення Mediator (який буде керувати взаємодією всіх класів)
    mediator = Mediator(cv_processor, kalman_filter, SHARED_DIR, "simple_motion") #"full_motion" / "simple_motion"

    # Створення Observer, який передає дані у Mediator
    observer_node = Observer(mediator, node_name='observer_simple') #'observer_full' / 'observer_simple'

    try:
        print("Усі вузли успішно запущені.")
        rclpy.spin(observer_node)  # крутимо головний цикл подій
    except KeyboardInterrupt:
        print("Зупинено користувачем (Ctrl+C)")
    finally:
        observer_node.mediator.analysis_results.save_all()
        observer_node.destroy_node()
        rclpy.shutdown()
        print("Усі вузли зупинені.")

if __name__ == '__main__':
    main()


"""
    executor = SingleThreadedExecutor()
    executor.add_node(observer_full)
    executor.add_node(observer_simple)

    try:
        print("Усі вузли успішно запущені.")
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.01)
            cv2.waitKey(1)
    except KeyboardInterrupt:
        print("Зупинено користувачем (Ctrl+C)")
    finally:
        # Акуратно зупиняємо executor і вузли
        executor.shutdown()
        rclpy.shutdown()

        # коректно закрити відео (і тільки потім зберігати графіки)
        mediator_full.close_writers()
        mediator_simple.close_writers()

        # Зберігаємо результати обох конвеєрів
        mediator_full.analysis_results.save_all()
        mediator_simple.analysis_results.save_all()

        # Звільняємо ресурси
        observer_full.destroy_node()
        observer_simple.destroy_node()
        print("Усі вузли зупинені.")\
"""