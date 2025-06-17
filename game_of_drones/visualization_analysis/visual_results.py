import cv2
import numpy as np

class VisualResults:
    def __init__(self):
        pass

    def hex_to_bgr(self, h: str) -> tuple[int, int, int]: #перетворення кольору з HEX-рядок у BGR
        h = h.lstrip('#')
        r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
        return (b, g, r)

    def draw(self, frame, real_point, predicted_point, radius=1):
        height, width = frame.shape[:2]
        # Перетворення нормалізованих координат у пікселі
        rx, ry = int(real_point[0] * width), int(real_point[1] * height)
        px, py = int(predicted_point[0] * width), int(predicted_point[1] * height)
        # Малюємо реальну точку
        cv2.circle(frame, (rx, ry), radius, (0, 0, 255), -1)  # RED
        # Малюємо прогнозовану точку
        cv2.circle(frame, (px, py), radius, (0, 255, 255), -1)  # YELLOW
        # Легенда
        cv2.putText(frame, "Real", (rx + 10, ry), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(frame, "Pred", (px + 10, py), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # у верхньому лівому куті (0,0)
        cv2.circle(frame, (0, 0), radius, (255, 0, 0), -1)  # синя точка
        cv2.putText(frame, "(0,0)", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

        # у нижньому правому куті (1,1) → (w-1,h-1)
        rx, ry = int(1 * width), int(1 * height)
        cv2.circle(frame, (rx, ry), radius, (0, 255, 0), -1)  # зелена точка
        cv2.putText(frame, "(1,1)", (width - 1 - 50, height - 1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        return frame

    def draw_pose(self, #Малює на кадрі
                frame: np.ndarray,
                centroid: tuple[float, float], #центроїд
                angle: float,
                contour: np.ndarray | None = None, #контур обʼєкта (якщо є)
                valid: bool = True,
                arrow_length: int = 20, #стрілку, що показує орієнтацію
                radius: int = 5
        ) -> np.ndarray:

        if not valid: # Ніяких позначок — просто повертаємо чистий кадр
            return frame

        h, w = frame.shape[:2]
        cx, cy = int(centroid[0]), int(centroid[1])
        # Якщо передали контур — малюємо його та його еліпс
        if contour is not None and len(contour) >= 5:
            cv2.drawContours(frame, [contour], -1, self.hex_to_bgr('#63AEBF'), 2)
            #ellipse = cv2.fitEllipse(contour)
            #cv2.ellipse(frame, ellipse, (0, 255, 0), 2)  # зелений еліпс

        # Малюємо центроїд
        cv2.circle(frame, (cx, cy), radius, self.hex_to_bgr('#F2AB6D'), -1)

        # Малюємо стрілку-орієнтацію
        end_x = int(cx + arrow_length * np.cos(angle))
        end_y = int(cy + arrow_length * np.sin(angle))
        cv2.arrowedLine(frame, (cx, cy), (end_x, end_y), (255, 255, 255), 2, tipLength=0.2)

        # Підписуємо кут в градусах
        angle_deg = int(np.degrees(angle))
        cv2.putText(frame, f"{angle_deg}", (cx + 5, cy - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.circle(frame, (cx, cy), 15, (0, 255, 0))

        return frame

    def draw_trajectory(self, frame, real_point, predicted_data_all, keys,
                        count_pred, method_pred, P = None, Q = None,
                        real_color_hex='#22e0ff', pred_color_hex='#faed27', radius=3):

        pred_trajectory = []
        h, w = frame.shape[:2]

        bgr_real = self.hex_to_bgr(real_color_hex)
        bgr_pred = self.hex_to_bgr(pred_color_hex)

        rx, ry = int(real_point[0] * w), int(real_point[1] * h)
        cv2.circle(frame, (rx, ry), radius, bgr_real, -1)

        values = [predicted_data_all[k] for k in keys if k in predicted_data_all]
        vec = np.array(values)
        pred_trajectory.append((vec[0], vec[1]))
        for i in range(count_pred):
            x_pred_step = method_pred(x = vec, dt = predicted_data_all["dt"], count_pred = i, P = P, Q = Q)
            # Зберігаємо нормовані координати майбутньої точки
            x_norm, y_norm = float(x_pred_step[0]), float(x_pred_step[1])
            pred_trajectory.append((x_norm, y_norm))
            # Малюємо точку на кадрі
            px, py = int(x_pred_step[0] * w), int(x_pred_step[1] * h)
            cv2.circle(frame, (px, py), radius, bgr_pred, -1)
        return frame, pred_trajectory
