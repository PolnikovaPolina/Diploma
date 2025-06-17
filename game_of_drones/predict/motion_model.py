import numpy as np
import math
from typing import Tuple
from abc import ABC, abstractmethod

class MotionModelStrategy(ABC):
    def __init__(self):
        self.history = []
        self.initialized = False
        self.beta_Q = 0.95

    def reset(self):
        """Скидає стан моделі для повторної ініціалізації"""
        self.history = []
        self.initialized = False

    @abstractmethod
    def initialize_state(self) -> Tuple[np.ndarray, float]:
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
    def count_element_measurement(self, valid: bool, measurement_frame: dict) -> np.ndarray:
        pass

    @abstractmethod
    def predict_steps(self, x: np.ndarray, dt: float, count_pred: int, P = None, Q = None) -> np.ndarray:
        pass

    def add_state_for_initialize(self, valid: bool, *data):
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

    # Генерує прогноз кроків за допомогою sigma-point-UKF на базі поточного стану x і коваріацій P,Q.
    def predict_steps_generate_sigma_points(self, x: np.ndarray, dt: float, count_pred: int, P, Q) -> np.ndarray:
        from GameOfDronesDev.game_of_drones.predict import unscented_filter_kalman
        x_prev = x
        p_prev = P

        for i in range(count_pred):
            sigma_points, W_m, W_c = unscented_filter_kalman.UnscentedFilterKalman.generate_sigma_points(x_prev, p_prev)
            # x_pred
            sigma_points_pred = [self.f_state(x_prev, dt) for x in sigma_points]
            x_next = np.zeros_like(x_prev)
            for i in range(len(sigma_points_pred)):
                x_next += W_m[i] * sigma_points_pred[i]
            # p_pred
            matrix_pred_without_noise = np.zeros_like(p_prev)
            for i in range(len(sigma_points_pred)):
                dx = (sigma_points_pred[i] - x_next).reshape(-1, 1)
                matrix_pred_without_noise += W_c[i] * (dx @ dx.T)
            p_next = matrix_pred_without_noise + Q

            # візуально станло краще, особливо перше коло, друге, яке здебіьшого гірше, непогане
            state_residual = x_prev.reshape(-1, 1) - x_next.reshape(-1, 1)
            Q = self.beta_Q * Q + (1 - self.beta_Q) * (state_residual @ state_residual.T)

            x_prev = x_next
            p_prev = p_next

        return x_prev

class FullMotionModel(MotionModelStrategy):

    def __init__(self):
        super().__init__()
        self.count = 9
        self.F = None
        self.last_dt = None

    def count_element_measurement(self, valid, measurement_frame):
        if not valid: # якщо об'єкт не був знайдений
            return None
        return np.atleast_2d([
            measurement_frame["centroid"][0],
            measurement_frame["centroid"][1],
            measurement_frame["angle"]
        ]).T # Повертає колонковий вектор [x, y, theta]^T

    def initialize_filter(self, image_width, image_height):
        P = np.eye(9) * 100
        Q = np.diag([
            1e-6,  # x
            1e-6,  # y
            1e-3,  # vx
            1e-3,  # vy
            1e-3,  # ax
            1e-3,  # ay
            1e-3,  # theta (кут)
            1e-3,  # omega (кутова швидкість)
            1e-3,  # alpha_omega (прискорення кута)
        ])  # Якщо Kalman “не встигає” за об’єктом — трохи збільши відповідні значення
        R = np.diag([
            (1 / image_width) ** 2,  # похибка x ~ 1 піксель
            (1 / image_height) ** 2,  # похибка y ~ 1 піксель
            (np.deg2rad(5)) ** 2  # якщо похибка кута ~5 градусів
        ])
        return P, Q, R

    def predict_steps(self, x, dt, count_pred, P = None, Q = None):
        x_prev = x
        for i in range(count_pred):
            theta = float(np.asarray(x_prev).flatten()[6])
            F = self.make_F(dt, theta)
            x_next = F @ x_prev
            x_prev = x_next

        return x_prev

    def make_F(self, dt, theta):
        F = np.eye(self.count)
        # x, y
        F[0, 2] = math.cos(theta) * dt # math.cos(theta) * v_x * dt
        F[0, 4] = 0.5 * math.cos(theta) * dt ** 2 # 0.5 * math.cos(theta) * a_x * dt ** 2
        F[1, 3] = math.sin(theta) * dt  # math.sin(theta) * v_y * dt
        F[1, 5] = 0.5 * math.sin(theta) * dt ** 2 # 0.5 * math.sin(theta) * a_y * dt ** 2
        F[2, 4] = dt # залежить від ax
        F[3, 5] = dt # залежить від ay
        # кутова координата залежить від omega, alpha
        F[6, 7] = dt # залежить від omega
        F[6, 8] = 0.5 * dt ** 2 # залежить від alpha
        # кутова швидкість залежить від alpha
        F[7, 8] = dt
        return F

    def f_state(self, x, dt):
        theta = float(np.asarray(x).flatten()[6])
        self.F = self.make_F(dt, theta)
        self.last_dt = dt
        return self.F @ x

    def h_state(self, x):
        h = np.asarray(x).flatten()  # shape (9,) x, y, v_x, v_y, a_x, a_y, theta, omega, alpha_omega
        return h[[0, 1, 6]].reshape(3, 1) # беремо лише 0-й, 1-й та 6-й елементи і повертаємо як колонковий вектор (3×1)

    def initialize_state(self):
        (z1, time_1), (z2, time_2) = self.history
        z1 = np.asarray(z1).flatten()  # [x1, y1, θ1]
        z2 = np.asarray(z2).flatten()  # [x2, y2, θ2]
        dt = time_2 - time_1
        if dt <= 0: raise ValueError(f"Неправильна послідовність часу. dt = {dt}")
        pos = z2[:2]
        vel = (z2[:2] - z1[:2]) / dt
        acc = np.zeros(2)  # [a_x, a_y]
        theta = z2[2]  # кут
        omega = (z2[2] - z1[2]) / dt  # кутова швидкість
        alpha = 0.0  # кутове прискорення
        print(f"\nx_1 = {z1[0]:.3f}, y_1 = {z1[1]:.3f}, theta_1 = {z1[2]:.3f}, t_1 = {time_1:.3f}\nx_2 = {z2[0]:.3f}, y_2 = {z2[1]:.3f}, theta_2 = {z2[2]:.3f}, t_2 = {time_2:.3f}\ndt = {dt:.3f}\n")
        # Збираємо в один вектор стану
        x0 = np.concatenate([pos, vel, acc, [theta, omega, alpha]])  # shape (9,)
        self.initialized = True
        return x0, dt

class SimpleMotionModel(MotionModelStrategy):

    def __init__(self):
        super().__init__()
        self.count = 6
        self.F = None
        self.last_dt = None

    def count_element_measurement(self, valid, measurement_frame):
        if not valid: # якщо об'єкт не був знайдений
            return None
        centroid = np.array(measurement_frame["centroid"])  # shape (2,)
        return np.array([centroid[0], centroid[1]]).reshape(2, 1) # shape (2,1)

    def initialize_filter(self, image_width, image_height):
        P = np.eye(self.count) * 100
        Q = np.diag([
            1e-6,  # x
            1e-6,  # y
            1e-4,  # vx
            1e-4,  # vy
            1e-5,  # ax
            1e-5  # ay
        ])  # Якщо Kalman “не встигає” за об’єктом — трохи збільши відповідні значення
        # та зменшити ковариацію вимірювань
        R = np.diag([
            (1 / image_width) ** 2,  # похибка x ~ 1 піксель
            (1 / image_height) ** 2  # похибка y ~ 1 піксель
        ])
        return P, Q, R

    def predict_steps(self, x, dt, count_pred, P = None, Q = None):
        x_prev = x
        F = self.make_F(dt)
        for i in range(count_pred):
            x_next = F @ x_prev
            x_prev = x_next
        return x_prev

    def make_F(self, dt):
        F = np.eye(self.count)
        # Для об'єкта
        F[0, 2] = dt
        F[0, 4] = 0.5 * dt ** 2
        F[1, 3] = dt
        F[1, 5] = 0.5 * dt ** 2
        F[2, 4] = dt
        F[3, 5] = dt
        return F

    def f_state(self, x, dt):
        self.F = self.make_F(dt)
        self.last_dt = dt
        return self.F.dot(x)

    def h_state(self, x):
        return np.array([
            [x[0]],  # x_object
            [x[1]]   # y_object
        ])

    def initialize_state(self):
        (z1, time_1), (z2, time_2) = self.history
        x1, y1 = z1.flatten() # [x1, y1]
        x2, y2 = z2.flatten()  # [x2, y2]
        xy_1 = np.array([x1, y1])
        xy_2 = np.array([x2, y2])
        dt = time_2 - time_1
        if dt <= 0: raise ValueError(f"Неправильна послідовність часу. dt = {dt}")
        pos = xy_2
        vel = (xy_2 - xy_1) / dt
        acc = np.zeros(2)  # [a_x, a_y]
        print(f"\nx_1 = {xy_1[0]:.3f}, y_1 = {xy_1[1]:.3f}, t_1 = {time_1:.3f}\nx_2 = {xy_2[0]:.3f}, y_2 = {xy_2[1]:.3f}, t_2 = {time_2:.3f}\ndt = {dt:.3f}\n")
        # Складаємо єдиний вектор стану
        x0 = np.concatenate([pos, vel, acc])
        self.initialized = True
        return x0, dt

"""
class FullMotionModel(MotionModelStrategy):

    def __init__(self):
        super().__init__()
        self.count = 9
        self.F = None
        self.B = None
        self.last_dt = None

    def count_element_measurement(self, measurement_frame):
        # Повертає колонковий вектор [x, y, theta]^T
        return np.atleast_2d([
            measurement_frame["centroid"][0],
            measurement_frame["centroid"][1],
            measurement_frame["angle"]
        ]).T

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

    def predict_steps(self, x, dt, count_pred, P = None, Q = None):
        x_prev = x
        for i in range(count_pred):
            theta = float(np.asarray(x_prev).flatten()[6])
            F = self.make_F(dt, theta)
            x_next = F @ x_prev
            x_prev = x_next

        return x_prev

    def make_F(self, dt, theta):
        F = np.eye(self.count)
        # x, y
        F[0, 2] = math.cos(theta) * dt # math.cos(theta) * v_x * dt
        F[0, 4] = 0.5 * math.cos(theta) * dt ** 2 # 0.5 * math.cos(theta) * a_x * dt ** 2
        F[1, 3] = math.sin(theta) * dt  # math.sin(theta) * v_y * dt
        F[1, 5] = 0.5 * math.sin(theta) * dt ** 2 # 0.5 * math.sin(theta) * a_y * dt ** 2
        # v_x, v_y
        F[2, 4] = dt # залежить від ax
        F[3, 5] = dt # залежить від ay
        # кутова координата залежить від omega, alpha
        F[6, 7] = dt # залежить від omega
        F[6, 8] = 0.5 * dt ** 2 # залежить від alpha
        # кутова швидкість залежить від alpha
        F[7, 8] = dt
        return F

    def make_B(self):
        B = -np.eye(self.count)
        return B

    def f_state(self, x, dt, u):
        u_arr = np.asarray(u).flatten()
        theta = float(np.asarray(x).flatten()[6])
        self.F = self.make_F(dt, theta)
        self.B = self.make_B()
        self.last_dt = dt
        return self.F @ x + self.B @ u_arr

    def h_state(self, x):
        h = np.asarray(x).flatten()  # shape (9,) x, y, v_x, v_y, a_x, a_y, theta, omega, alpha_omega
        return h[[0, 1, 6]].reshape(3, 1) # беремо лише 0-й, 1-й та 6-й елементи і повертаємо як колонковий вектор (3×1)

    def initialize_state(self, u):
        u = np.asarray(u).flatten()  # [u_x, u_y, u_vx, u_vy, u_ax, u_ay, u_th, u_w, u_aw]
        (z1, time_1), (z2, time_2) = self.history
        z1 = np.asarray(z1).flatten()  # [x1, y1, θ1]
        z2 = np.asarray(z2).flatten()  # [x2, y2, θ2]
        dt = time_2 - time_1
        if dt <= 0: raise ValueError(f"Неправильна послідовність часу. dt = {dt}")
        pos = z2[:2] - u[:2]  # [x2 - u_x, y2 - u_y]
        vel = (z2[:2] - z1[:2]) / dt - u[2:4]  # [v_x, v_y]
        acc = np.zeros(2)  # [a_x, a_y]
        theta = z2[2] - u[6]  # скоригований кут
        omega = (z2[2] - z1[2]) / dt - u[7]  # кутова швидкість
        alpha = 0.0  # кутове прискорення
        print(f"\nx_1 = {z1[0]:.3f}, y_1 = {z1[1]:.3f}, theta_1 = {z1[2]:.3f}, t_1 = {time_1:.3f}\nx_2 = {z2[0]:.3f}, y_2 = {z2[1]:.3f}, theta_2 = {z2[2]:.3f}, t_2 = {time_2:.3f}\ndt = {dt:.3f}\n")
        # Збираємо в один вектор стану
        x0 = np.concatenate([pos, vel, acc, [theta, omega, alpha]])  # shape (9,)
        self.initialized = True
        return x0, dt
class SimpleMotionModel(MotionModelStrategy):

    def __init__(self):
        super().__init__()
        self.count = 12
        self.F = None
        self.B = None
        self.last_dt = None

    def count_element_measurement(self, valid, measurement_frame, u):
        if not valid: # якщо об'єкт не був знайдений
            return None
        centroid = np.array(measurement_frame["centroid"])  # shape (2,)
        centroid -= np.asarray(u).flatten()[:2]
        return np.array([centroid[0], centroid[1], u[0], u[1]]).reshape(4, 1) # shape (4,1)

    def initialize_filter(self, image_width, image_height):
        P = np.eye(self.count) * 100
        Q = np.diag([
            1e-6,  # x
            1e-6,  # y
            1e-4,  # vx
            1e-4,  # vy
            1e-5,  # ax
            1e-5,  # ay
            1e-6,  # u_x
            1e-6,  # u_y
            1e-4,  # u_vx
            1e-4,  # u_vy
            1e-5,  # u_ax
            1e-5  # u_ay
        ])  # Якщо Kalman “не встигає” за об’єктом — трохи збільши відповідні значення
        # та зменшити ковариацію вимірювань
        R = np.diag([
            (1 / image_width) ** 2,  # похибка x ~ 1 піксель
            (1 / image_height) ** 2,  # похибка y ~ 1 піксель
            (2 / image_width) ** 2,  # похибка u_x ~ 2 піксель
            (2 / image_height) ** 2  # похибка u_y ~ 2 піксель
        ])
        return P, Q, R

    def predict_steps(self, x, dt, count_pred, P = None, Q = None):
        x_prev = x
        F = self.make_F(dt)
        for i in range(count_pred):
            x_next = F @ x_prev
            x_prev = x_next
        return x_prev

    def make_F(self, dt):
        F = np.eye(self.count)
        # Для об'єкта
        F[0, 2] = dt
        F[0, 4] = 0.5 * dt ** 2
        F[1, 3] = dt
        F[1, 5] = 0.5 * dt ** 2
        F[2, 4] = dt
        F[3, 5] = dt
        # Для об'єкта, якщо камера рухається
        F[0, 6] = -1 # x_pos: -u_x
        F[1, 7] = -1 # y_pos: -u_y
        F[2, 8] = -1 # v_x: -u_v_x
        F[3, 9] = -1 # v_y: -u_v_y
        F[4, 10] = -1 # a_x: -u_a_x
        F[5, 11] = -1 # a_y: -u_a_y
        # Для камери
        F[6, 8] = dt # u_x: +u_v_x*dt
        F[6, 10] = 0.5 * dt ** 2 # u_x: +0.5 * u_a_x * dt^2
        F[7, 9] = dt # u_y: +u_v_y*dt
        F[7, 11] = 0.5 * dt ** 2  # u_y: +0.5 * u_a_y * dt^2
        F[10, 8] = dt # u_v_x: + u_a_x * dt
        F[11, 9] = dt # u_v_y: + u_a_y * dt
        return F

    def make_B(self, dt):
        B = -np.eye(self.count)
        #B[0, 2] = - dt
        #B[0, 4] = - 0.5 * dt ** 2
        #B[1, 3] = - dt
        #B[1, 5] = - 0.5 * dt ** 2
        #B[2, 4] = - dt
        #B[3, 5] = - dt
        return B

    def f_state(self, x, dt):
        #u6 = np.asarray(u).flatten()[:self.count]  # shape (6,)
        #u6[2:] = 0
        self.F = self.make_F(dt)
        #self.B = self.make_B(dt)
        self.last_dt = dt
        return self.F.dot(x)

    def h_state(self, x):
        return np.array([
            [x[0] - x[6]],  # x_object - u_x (зміщення об'єкта відносно камери)
            [x[1] - x[7]],   # y_object - u_y
            [x[6]],
            [x[7]]
        ])
        #return x[[0, 1, 6, 7]].reshape(-1, 1) #x, y

    def initialize_state(self):

        (z1, time_1), (z2, time_2) = self.history
        x1, y1, u1_x, u1_y = z1.flatten() # [x1, y1]
        x2, y2, u2_x, u2_y = z2.flatten()  # [x2, y2]
        xy_1 = np.array([x1, y1])
        xy_2 = np.array([x2, y2])
        u = np.array([u2_x, u2_y])

        dt = time_2 - time_1
        if dt <= 0: raise ValueError(f"Неправильна послідовність часу. dt = {dt}")

        pos = xy_2 - u # - u[:2]  # [x2-u_x, y2-u_y]
        vel = (xy_2 - xy_1) / dt # - u[2:4]  # [v_x, v_y]
        acc = np.zeros(2)  # [a_x, a_y]

        u_vx = u_vy = u_ax = u_ay = 0

        print(f"\nx_1 = {xy_1[0]:.3f}, y_1 = {xy_1[1]:.3f}, t_1 = {time_1:.3f}\nx_2 = {xy_2[0]:.3f}, y_2 = {xy_2[1]:.3f}, t_2 = {time_2:.3f}\ndt = {dt:.3f}\n")

        # Складаємо єдиний вектор стану
        x0 = np.concatenate([pos, vel, acc, u, np.array([u_vx, u_vy, u_ax, u_ay])])
        self.initialized = True
        return x0, dt
"""