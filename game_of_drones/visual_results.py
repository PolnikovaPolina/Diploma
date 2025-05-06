import cv2
import numpy as np

class VisualResults:
    def __init__(self):
        pass


    def hex_to_bgr(self, h: str) -> tuple[int, int, int]: #перетворення кольору з HEX-рядок у BGR
        h = h.lstrip('#')
        r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
        return (b, g, r)

    def draw(self, frame, real_point, predicted_point, radius=5):
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
        # Переводимо нормалізовані координати в пікселі
        cx, cy = int(centroid[0] * w), int(centroid[1] * h)

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
        cv2.arrowedLine(frame, (cx, cy), (end_x, end_y), (0, 0, 0), 2, tipLength=0.2)

        # Підписуємо кут в градусах
        angle_deg = int(np.degrees(angle))
        cv2.putText(frame, f"{angle_deg}", (cx + 5, cy - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        return frame