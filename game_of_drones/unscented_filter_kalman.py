import numpy as np
from motion_model import MotionModelStrategy

class UnscentedFilterKalman:

    def __init__(self, motion_model: MotionModelStrategy):
        self.motion_model = motion_model
        self.dt = None
        self.t = None
        self.x = None
        self.z = None
        self.P = None
        self.Q = None
        self.R = None
        self.image_width = None
        self.image_height = None
        self.alpha_R = 0.95
        self.beta_Q = 0.95

    def predict(self, measurement_frame):

        self.t = measurement_frame["time"]
        self.dt = measurement_frame["dt"]
        valid = measurement_frame["valid"]
        self.image_width = measurement_frame["image_width"]
        self.image_height = measurement_frame["image_height"]

        self.z = self.motion_model.count_element_measurement(measurement_frame)
        init_state = self.motion_model.add_state_for_initialize(valid, self.z, self.t)

        if init_state is not None:
            self.x, self.dt = init_state
            print(f"Initial State:\n{np.round(self.x, 3)}")
            print(f"dt: {self.dt:.3f}")
            self.P, self.Q, self.R = self.motion_model.initialize_filter(self.image_width, self.image_height)
            return {
                "time": self.t,
                "predicted": self.x.tolist(),  # [x, y, v_x, v_y, a_x, a_y ,θ, v_θ, a_θ]
                "image_width": self.image_width,
                "image_height": self.image_height }

        if self.x is None or self.P is None:
            return None  # ще не ініціалізовано

        # Обчислення сігма точок
        sigma_points, W_m, W_c = self.generate_sigma_points(self.x, self.P)

        # Прогноз через сігма точки потім усереднення результату

        # x_pred
        sigma_points_pred = [self.motion_model.f_state(x, self.dt) for x in sigma_points]
        x_pred = np.zeros_like(self.x)
        for i in range(len(sigma_points_pred)):
            x_pred += W_m[i] * sigma_points_pred[i]

        # p_pred
        matrix_pred_without_noise = np.zeros_like(self.P)
        for i in range(len(sigma_points_pred)):
            dx = (sigma_points_pred[i] - x_pred).reshape(-1, 1)
            matrix_pred_without_noise += W_c[i] * (dx @ dx.T)
        p_pred = matrix_pred_without_noise + self.Q

        if not valid:
            print("Вимірювання не доступне. Лише прогноз без корекції.")
            self.x = x_pred
            self.P = p_pred

        else:
            # z_pred
            z_sigma_points_pred = [self.motion_model.h_state(x) for x in sigma_points_pred]
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
            y = (self.z.reshape(-1) - z_pred.reshape(-1))  # Обчислення нев'язки
            self.x = x_pred + (K @ y).reshape(-1)  # Оновлення стану
            self.P = p_pred - K @ S @ K.T  # Оновлення коваріації

            # print(f"x_pred: {self.x.tolist()}")

            # Оновлення R (коваріація шуму вимірювань)
            measurement_residual = self.z.reshape(-1, 1) - z_pred
            self.R = self.alpha_R * self.R + (1 - self.alpha_R) * (measurement_residual @ measurement_residual.T)

            # Оновлення Q (коваріація процесного шуму)
            state_residual = self.x.reshape(-1, 1) - x_pred.reshape(-1, 1)
            self.Q = self.beta_Q * self.Q + (1 - self.beta_Q) * (state_residual @ state_residual.T)

            # print(f"P = \n{self.P}")
            # print(f"Q = \n{self.Q}")
            # print(f"R = \n{self.R}")

        return {
            "time": self.t,
            "predicted": self.x.tolist(),  # [x, y, v_x, v_y, a_x, a_y ,θ, v_θ, a_θ]
            "image_width": self.image_width,
            "image_height": self.image_height
        }

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

        W_m = np.full(2 * n + 1, 1 / (2 * c))  # усереднення сигма-точок для отримання прогнозу стану
        W_c = np.full(2 * n + 1, 1 / (2 * c))  # обчислення ковариації P після поширення сигма-точок
        W_m[0] = lam / c
        W_c[0] = lam / c + (1 - alpha ** 2 + beta)

        return sigma_points, W_m, W_c
