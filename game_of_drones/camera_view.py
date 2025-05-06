import rclpy
from numpy.matrixlib.defmatrix import matrix
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
from sensor_msgs.msg import CompressedImage  # Import the necessary message type
# from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray

import subprocess
import os
import time
import sys

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

        self.predicted_publisher = self.create_publisher(
            Float32MultiArray,
            '/predicted_pixel_pose',
            qos_profile_unity
        )

        self.prev_time = None
        self.init_history = []
        self.initialized = False
        self.image_height = None
        self.image_width = None
        self.Q = None
        self.R = None
        self.P = None
        self.x = None
        self.dt = None
        self.z = None
        self.alpha_R = 0.95  # коефіцієнт згладжування для вимірювань (0.9 — сильне згладжування)
        self.beta_Q = 0.95  # коефіцієнт згладжування для процесного шуму

    def initialize_state(self, centroid_1, theta_1, time_1,
                               centroid_2, theta_2, time_2):

        dt = time_2 - time_1
        self.dt = dt

        if dt <= 0:
            raise ValueError(f"Неправильна послідовність часу. dt = {dt}")

        x1, y1 = centroid_1
        x2, y2 = centroid_2

        v_x = (x2 - x1) / dt
        v_y = (y2 - y1) / dt
        a_x = 0.0
        a_y = 0.0

        omega = (theta_2 - theta_1) / dt # кутова швидкість
        alpha_omega = 0.0  # кутове прискорення
        theta = theta_2 # поточна орієнтація:

        print(f"\nx_1 = {x1}, y_1 = {y1}, theta_1 = {theta_1}, t_1 = {time_1}\nx_2 = {x2}, y_2 = {y2}, theta_2 = {theta_2}, t_2 = {time_2}\ndt = {dt}\n")

        return np.array([x2, y2, v_x, v_y, a_x, a_y,
                         theta, omega, alpha_omega])

    def collect_initialization_data(self, centroid, theta, timestamp):
        """
        Збирає два перші (центроїд, кут, час) й обчислює первинний стан.
        Повертає np.array початкового стану, коли зібрано два виміри,
        або None, якщо ще недостатньо даних.
        """
        if self.initialized:
            return None

        self.init_history.append((centroid, theta, timestamp))
        if len(self.init_history) < 2:
            return None

        # Розпаковуємо два перші елементи
        (c1, th1, t1), (c2, th2, t2) = self.init_history
        self.initialized = True
        return self.initialize_state(c1, th1, t1, c2, th2, t2) # Викликаємо initialize_state для обчислення вектора стану

    def generate_sigma_points(self, x, P, alpha=1e-3, beta=2, kappa=0):
        n = x.shape[0]
        lam = alpha ** 2 * (n + kappa) - n
        c = n + lam
        sqrt_P = np.linalg.cholesky(P * c)

        sigma_points = np.zeros((2 * n + 1, n))
        sigma_points[0] = x
        for i in range(n):
            sigma_points[i + 1] = x + sqrt_P[:, i]
            sigma_points[n + i + 1] = x - sqrt_P[:, i]

        W_m = np.full(2 * n + 1, 1 / (2 * c)) # усереднення сигма-точок для отримання прогнозу стану
        W_c = np.full(2 * n + 1, 1 / (2 * c)) # обчислення ковариації P після поширення сигма-точок
        W_m[0] = lam / c
        W_c[0] = lam / c + (1 - alpha ** 2 + beta)

        return sigma_points, W_m, W_c

    def f_state(self, x, dt):
        x_pos, y_pos, v_x, v_y, a_x, a_y, theta, omega, alpha_omega = x.flatten()

        v_x += a_x * dt
        v_y += a_y * dt

        omega += alpha_omega * dt

        theta +=  omega*dt+0.5*alpha_omega*dt**2

        x_pos += math.cos(theta) * v_x * dt + 0.5 * math.cos(theta) * a_x * dt ** 2
        y_pos += math.sin(theta) * v_y * dt + 0.5 * math.sin(theta) * a_y * dt ** 2

        return np.array([x_pos, y_pos, v_x, v_y, a_x, a_y,
                         theta, omega, alpha_omega])

    def h_state(self, x):
        x, y, v_x, v_y, a_x, a_y, theta, omega, alpha_omega = x.flatten()
        return np.array([[x], [y], [theta]])

    def camera_image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        self.image_height, self.image_width = image_np.shape[:2]

        # Отримаємо точний час
        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec
        t = sec + nsec * 1e-9
        if self.prev_time is None: dt = 0.0
        else: self.dt = t - self.prev_time
        self.prev_time = t

        # Compute the gradient image using Sobel operator
        gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
        grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        gradient = cv2.magnitude(grad_x, grad_y)
        gradient = cv2.convertScaleAbs(gradient)
        
        # Display the gradient image
        #cv2.imshow('Gradient Image', gradient)

        # Обчислити центроїд та орієнтацію

        _, binary = cv2.threshold(gradient, 5, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        image_test = image_np.copy()
        try:
            largest_contour = max(contours, key=cv2.contourArea)
        except ValueError:
            print("Контурів не знайдено")

        contour_object = np.vstack(largest_contour)

        M = cv2.moments(contour_object) # Отримуємо моменти
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centroid = (cx / self.image_width, cy / self.image_height) # нормуємо координати
            angle = 0.5 * np.arctan2(2 * M['mu11'], M['mu20'] - M['mu02']) # Орієнтація
            angle = np.mod(angle + np.pi, 2 * np.pi) - np.pi # ∈[–π,+π]
            angle_degrees = np.degrees(angle)

            x_norm, y_norm = centroid
            self.z = np.array([[x_norm], [y_norm], [angle]])

            # Візуалізація
            cv2.drawContours(image_test, largest_contour, -1, (0, 0, 255), 2)
            cv2.circle(image_test, (cx, cy), 5, (255, 255, 0), -5)
            cv2.arrowedLine(image_test, (cx, cy),
                            (
                                int(cx + 20 * np.cos(angle)),
                                int(cy + 20 * np.sin(angle))),
                            (0, 0, 0), 2, tipLength=0.2)

            if len(contour_object) >= 5:
                ellipse = cv2.fitEllipse(contour_object)
                cv2.ellipse(image_test, ellipse, (0, 0, 0), 2)

        else:
            self.z = None
            print("Момент m00 = 0, контур занадто маленький або дивний")


        #Ініціалізація початкових значень та матриць невизначеності

        init_state = self.collect_initialization_data(centroid, angle, t)
        if init_state is not None:
            #print("Початковий стан зібрано:", init_state.flatten())

            self.Q = np.diag([
                1e-6,  # x
                1e-6,  # y
                1e-4,  # vx
                1e-4,  # vy
                1e-5,  # ax
                1e-5,  # ay
                1e-4,  # theta (кут)
                1e-4,  # omega (кутова швидкість)
                1e-5,  # alpha_omega (прискорення кута)
            ]) # Якщо Kalman “не встигає” за об’єктом — трохи збільши відповідні значення

            self.R = np.diag([
                (1 / self.image_width) ** 2,  # похибка x ~ 1 піксель
                (1 / self.image_height) ** 2,  # похибка y ~ 1 піксель
                (np.deg2rad(5)) ** 2  # якщо похибка кута ~5 градусів
            ])

            self.P = np.array([
                [10.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0],
                [0.0, 10.0, 0.0, 0.0, 0, 0, 0, 0, 0],
                [0.0, 0.0, 100.0, 0.0, 0, 0, 0, 0, 0],
                [0.0, 0.0, 0.0, 100.0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 10, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 10, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 10, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 10, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 10],
            ])

            self.x = init_state

            return

        if self.x is None or self.P is None:
            return

        # Обчислення сігма точок
        sigma_points, W_m, W_c = self.generate_sigma_points(self.x, self.P)

        # Прогноз через сігма точки потім усереднення результату

        # x_pred
        sigma_points_pred = [self.f_state(x, self.dt) for x in sigma_points]
        x_pred = np.zeros_like(self.x)
        for i in range(len(sigma_points_pred)):
            x_pred += W_m[i] * sigma_points_pred[i]

        # p_pred
        matrix_pred_without_noise = np.zeros_like(self.P)
        for i in range(len(sigma_points_pred)):
            dx = (sigma_points_pred[i] - x_pred).reshape(-1, 1)
            matrix_pred_without_noise += W_c[i] * (dx @ dx.T)
        p_pred = matrix_pred_without_noise + self.Q

        if self.z is None:
            print("Вимірювання не доступне. Лише прогноз без корекції.")
            self.x = x_pred
            self.P = p_pred

        else:
            # z_pred
            z_sigma_points_pred = [self.h_state(x) for x in sigma_points_pred]
            z_pred = np.zeros_like(z_sigma_points_pred[0])
            for i in range(len(z_sigma_points_pred)):
                z_pred += W_m[i] * z_sigma_points_pred[i]

            # фільтр Калмана

            # S - Коваріація вимірювання
            matrix_s_without_noise = np.zeros_like(self.R)
            for i in range(len(z_sigma_points_pred)):
                dz = (z_sigma_points_pred[i] - z_pred).reshape(-1, 1)
                matrix_s_without_noise += W_c[i] * (dz @ dz.T)
            S = matrix_s_without_noise + self.R

            # P_xz - Крос-коваріація між станом та вимірюванням
            P_xz = np.zeros((x_pred.shape[0], z_pred.shape[0]))
            for i in range(len(sigma_points_pred)):
                dx = (sigma_points_pred[i] - x_pred).reshape(-1, 1)
                dz = z_sigma_points_pred[i] - z_pred
                P_xz += W_c[i] * (dx @ dz.T)

            K = P_xz @ np.linalg.inv(S)  # Обчислення Kalman Gain
            y = (self.z.reshape(-1) - z_pred.reshape(-1)) # Обчислення нев'язки
            self.x = x_pred + (K @ y).reshape(-1) # Оновлення стану
            self.P = p_pred - K @ S @ K.T # Оновлення коваріації

            #print(f"x_pred: {self.x.tolist()}")

            # Оновлення R (коваріація шуму вимірювань)
            measurement_residual = self.z.reshape(-1, 1) - z_pred
            self.R = self.alpha_R * self.R + (1 - self.alpha_R) * (measurement_residual @ measurement_residual.T)

            # Оновлення Q (коваріація процесного шуму)
            state_residual = self.x.reshape(-1, 1) - x_pred.reshape(-1, 1)
            self.Q = self.beta_Q * self.Q + (1 - self.beta_Q) * (state_residual @ state_residual.T)

            #print(f"P = \n{self.P}")
            #print(f"Q = \n{self.Q}")
            #print(f"R = \n{self.R}")

        x_predicted = self.x[0] * self.image_width
        y_predicted = self.x[1] * self.image_height

        msg_out = Float32MultiArray()
        msg_out.data = [self.x[0], self.x[1]]
        self.predicted_publisher.publish(msg_out)

        cv2.circle(image_np, (int(x_predicted), int(y_predicted)), 5, (0, 255, 255), -1)

        cv2.imshow('Camera Image', image_np)
        #cv2.imshow("Test", image_test)

        cv2.waitKey(1) 

def main(args=None):
    rclpy.init(args=args)
    node = CameraView()

    ANALYSIS_SCRIPT = "analysis_results.py"
    if not os.path.exists(ANALYSIS_SCRIPT):
        print(f"Файл {ANALYSIS_SCRIPT} не знайдено!")
    else:
        print(f"Запускаємо {ANALYSIS_SCRIPT}...")
        subprocess.Popen(["python3", ANALYSIS_SCRIPT], stdout=sys.stdout, stderr=sys.stderr)
        time.sleep(1) 

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Завершено")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
