from abc import ABC, abstractmethod
import cv2
import numpy as np

class ContourStrategy(ABC):
    @abstractmethod
    def find(self, image: np.ndarray) -> np.ndarray:
        pass

class LargestContour(ContourStrategy):
    def find(self, image: np.ndarray) -> np.ndarray:
        contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        try:
            largest_contour = max(contours, key=cv2.contourArea)
        except ValueError:
            print("Контурів не знайдено")
            return None
        return largest_contour