import numpy as np
import math
from typing import Tuple
from abc import ABC, abstractmethod

class MotionModelStrategy(ABC):
    def __init__(self):
        self.history = []
        self.initialized = False

    def reset(self):
        """Скидає стан моделі для повторної ініціалізації"""
        self.history = []
        self.initialized = False

    @abstractmethod
    def initialize_state(self, x: np.ndarray, dt: float) -> Tuple[np.ndarray, float]:
        pass
    @abstractmethod
    def f_state(self, x: np.ndarray, dt: float) -> np.ndarray:
        pass
    @abstractmethod
    def h_state(self, x: np.ndarray) -> np.ndarray:
        pass
    @abstractmethod
    def initialize_filter(self, image_width: int, image_height: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        pass
    @abstractmethod
    def count_element_measurement(self, measurement_frame: dict) -> np.ndarray:
        pass

    def add_state_for_initialize(self, valid, *data):
        if self.initialized:
            return None
        if len(self.history) >= 2:
            raise ValueError(
                "InitPoint: надто багато точок! Ініціалізація повинна базуватись лише на перших двох вимірюваннях.")

        if valid:
            self.history.append(data)
            if len(self.history) == 2:
                return self.initialize_state()
        return None

class FullMotionModel(MotionModelStrategy):

    def count_element_measurement(self, measurement_frame):
        centroid = measurement_frame["centroid"]
        angle = measurement_frame["angle"]
        x_norm, y_norm = centroid
        return np.array([[x_norm], [y_norm], [angle]])

    def initialize_filter(self, image_width, image_height):
        P = np.eye(9) * 100
        Q = np.diag([
            1e-6,  # x
            1e-6,  # y
            1e-4,  # vx
            1e-4,  # vy
            1e-5,  # ax
            1e-5,  # ay
            1e-4,  # theta (кут)
            1e-4,  # omega (кутова швидкість)
            1e-5,  # alpha_omega (прискорення кута)
        ])  # Якщо Kalman “не встигає” за об’єктом — трохи збільши відповідні значення
        R = np.diag([
            (1 / image_width) ** 2,  # похибка x ~ 1 піксель
            (1 / image_height) ** 2,  # похибка y ~ 1 піксель
            (np.deg2rad(5)) ** 2  # якщо похибка кута ~5 градусів
        ])
        return P, Q, R

    def f_state(self, x, dt):
        x_pos, y_pos, v_x, v_y, a_x, a_y, theta, omega, alpha_omega = x.flatten()

        x_pos += math.cos(theta) * v_x * dt + 0.5 * math.cos(theta) * a_x * dt ** 2
        y_pos += math.sin(theta) * v_y * dt + 0.5 * math.sin(theta) * a_y * dt ** 2
        v_x += a_x * dt
        v_y += a_y * dt
        omega += alpha_omega * dt
        theta += omega * dt + 0.5 * alpha_omega * dt ** 2

        return np.array([x_pos, y_pos, v_x, v_y, a_x, a_y,
                         theta, omega, alpha_omega])

    def h_state(self, x):
        x, y, v_x, v_y, a_x, a_y, theta, omega, alpha_omega = x.flatten()
        return np.array([[x], [y], [theta]])

    def initialize_state(self):
        (z1, time_1), (z2, time_2) = self.history
        x1, y1, theta_1 = z1.flatten()
        x2, y2, theta_2 = z2.flatten()
        dt = time_2 - time_1
        if dt <= 0: raise ValueError(f"Неправильна послідовність часу. dt = {dt}")
        v_x = (x2 - x1) / dt
        v_y = (y2 - y1) / dt
        a_x = a_y = alpha_omega = 0.0  # прискорення
        omega = (theta_2 - theta_1) / dt  # кутова швидкість
        theta = theta_2  # поточна орієнтація:
        #print(f"\nx_1 = {x1:.3f}, y_1 = {y1:.3f}, theta_1 = {theta_1:.3f}, t_1 = {time_1:.3f}\nx_2 = {x2:.3f}, y_2 = {y2:.3f}, theta_2 = {theta_2:.3f}, t_2 = {time_2:.3f}\ndt = {dt:.3f}\n")
        self.initialized = True
        return np.array([x2, y2, v_x, v_y, a_x, a_y, theta, omega, alpha_omega]), dt

class SimpleMotionModel(MotionModelStrategy):

    def count_element_measurement(self, measurement_frame):
        centroid = measurement_frame["centroid"]
        x_norm, y_norm = centroid
        return np.array([[x_norm], [y_norm]])

    def initialize_filter(self, image_width, image_height):
        P = np.eye(6) * 100
        Q = np.diag([
            1e-6,  # x
            1e-6,  # y
            1e-4,  # vx
            1e-4,  # vy
            1e-5,  # ax
            1e-5  # ay
        ])  # Якщо Kalman “не встигає” за об’єктом — трохи збільши відповідні значення
        R = np.diag([
            (1 / image_width) ** 2,  # похибка x ~ 1 піксель
            (1 / image_height) ** 2  # похибка y ~ 1 піксель
        ])
        return P, Q, R

    def f_state(self, x, dt):
        x_pos, y_pos, v_x, v_y, a_x, a_y = x.flatten()
        x_pos += v_x * dt + 0.5 * a_x * dt ** 2
        y_pos += v_y * dt + 0.5 * a_y * dt ** 2
        v_x += a_x * dt
        v_y += a_y * dt

        return np.array([x_pos, y_pos, v_x, v_y, a_x, a_y])

    def h_state(self, x):
        x, y, v_x, v_y, a_x, a_y = x.flatten()
        return np.array([[x], [y]])

    def initialize_state(self):
        (z1, time_1), (z2, time_2) = self.history
        x1, y1 = z1.flatten()
        x2, y2 = z2.flatten()
        dt = time_2 - time_1
        if dt <= 0: raise ValueError(f"Неправильна послідовність часу. dt = {dt}")
        v_x = (x2 - x1) / dt
        v_y = (y2 - y1) / dt
        a_x = a_y  = 0.0  # прискорення
        print(
            f"\nx_1 = {x1:.3f}, y_1 = {y1:.3f}, t_1 = {time_1:.3f}\nx_2 = {x2:.3f}, y_2 = {y2:.3f}, t_2 = {time_2:.3f}\ndt = {dt:.3f}\n")
        self.initialized = True
        return np.array([x2, y2, v_x, v_y, a_x, a_y]), dt