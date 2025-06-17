import cv2
import numpy as np

from GameOfDronesDev.game_of_drones.detection.object_detection import DetectionStrategy
from GameOfDronesDev.game_of_drones.detection.contour_detection import ContourStrategy
from my_interface.msg import CameraStamped

class ComputerVision:

    def __init__(self, detection_strategy: DetectionStrategy, contour_strategy: ContourStrategy):

        self.detection_strategy = detection_strategy
        self.contour_strategy = contour_strategy

        self.prev_time = None
        self.image_height = None
        self.image_width = None

    def get_measurement_frame(self, msg):

        # Отримання зображення з ROS-повідомлення
        np_arr = np.frombuffer(msg.image.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        self.image_height, self.image_width = frame.shape[:2]

        # Часові мітки для синхронізації

        time_str = msg.header.frame_id
        t = float(time_str)
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = t - self.prev_time
        self.prev_time = t

        # Стратегія обробки
        image = self.detection_strategy.process(frame)

        # Стратегія знаходження контуру
        largest_contour = self.contour_strategy.find(image)
        if largest_contour is None or len(largest_contour) <= 5:
            #print("Контур не знайдено")
            return {
                "time": t,
                "dt": dt,
                "valid": False,
                "centroid": (None, None),
                "angle": None,
                "contour": None,
                "unity_payload": msg.status.data,
                "image_width": self.image_width,
                "image_height": self.image_height,
                "frame": frame
            }

        contour_object = np.vstack(largest_contour)
        image_visualization = frame.copy()

        # Обчислити центроїд та орієнтацію через моменти

        M = cv2.moments(contour_object)  # Отримуємо моменти
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"]) # отримаємо координати одразу не нормалізуємо через візуалізацію
            cy = int(M["m01"] / M["m00"]) # нормалізуємо
            centroid = (cx / self.image_width, cy / self.image_height)
            #angle = 0.5 * np.arctan2(2 * M['mu11'], M['mu20'] - M['mu02'])  # Орієнтація
            #angle = np.mod(angle + np.pi, 2 * np.pi) - np.pi  # ∈[–π,+π]

            eps = 1e-7  # захист від ділення на 0
            den = M['mu20'] - M['mu02']
            if abs(den) < eps:  # майже симетрія
                angle = np.pi / 4 if M['mu11'] > 0 else -np.pi / 4
            else:
                angle = 0.5 * np.arctan2(2 * M['mu11'], den)

            # приводимо у [-π, π]
            angle = (angle + np.pi) % (2 * np.pi) - np.pi # ∈[–π,+π]
            angle_degrees = np.degrees(angle)

            return {
                "time": t,
                "dt": dt,
                "valid": True,
                "centroid": centroid,
                "angle": angle,
                "contour": contour_object,
                "unity_payload": msg.status.data,
                "image_width": self.image_width,
                "image_height": self.image_height,
                "frame": frame
            }


        else:
            print("Момент m00 = 0, контур занадто маленький або дивний")
            return {
                    "time": t,
                    "dt": dt,
                    "valid": False,
                    "centroid": None,
                    "angle": None,
                    "contour": None,
                    "image_width": self.image_width,
                    "image_height": self.image_height,
                    "frame": frame
                }