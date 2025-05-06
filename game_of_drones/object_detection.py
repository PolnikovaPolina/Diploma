from abc import ABC, abstractmethod
import cv2
import numpy as np

class DetectionStrategy(ABC):
    @abstractmethod
    def process(self, image: np.ndarray) -> np.ndarray:
        pass

class SobelDetection(DetectionStrategy):
    def process(self, image: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        gradient = cv2.magnitude(grad_x, grad_y)
        gradient = cv2.convertScaleAbs(gradient)
        _, binary = cv2.threshold(gradient, 5, 255, cv2.THRESH_BINARY)
        return binary

# NotImplementedError - спеціальна команда, яка явно вказує, що метод ще
# не реалізовано. Вона часто використовується в абстрактних класах або
# інтерфейсах, коли ти хочеш змусити дочірні класи реалізувати цей метод
# самостійно.
# Однак було вирішено реалізовувати через @abstractmethod, а не raise NotImplementedError, хоча можна і через нього
