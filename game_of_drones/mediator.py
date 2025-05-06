import os, cv2, csv, atexit # автоматично закриття cvs файлу
import pandas as pd

from computer_vision import ComputerVision
from unscented_filter_kalman import UnscentedFilterKalman
from analysis_results import AnalysisResults
from visual_results import VisualResults
from camera_controller import CameraController

class Mediator:
    def __init__(self, cv_processor, kalman_filter, share_dir, name_file):

        self.share_dir = share_dir
        self.name_file = name_file

        # Підготуємо VideoWriter, але розмір кадру поки невідомий:

        # Підготуємо шифрек і фпс один раз
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.fps = 20.0

        # Перший відео-файл (з центроїдом та орієнтацією)
        self.pose_path = os.path.join(self.share_dir, f'pose_{self.name_file}.avi')
        self.pose_writer = None

        # Другий відео-файл (з real vs pred)
        self.cmp_path = os.path.join(self.share_dir, f'compare_{self.name_file}.avi')
        self.cmp_writer = None

        # CSV таблиця
        self.csv_path = os.path.join(self.share_dir, f'results_{self.name_file}.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        atexit.register(self.csv_file.close)  # файл закриється при виході з програми
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time',
            'real_x', 'real_y',
            'pred_x', 'pred_y', 'pred_v_x', 'pred_v_y', 'pred_a_x', 'pred_a_y', 'pred_theta', 'pred_theta_w',
            'pred_theta_a'
        ])

        self.cv_processor = cv_processor
        self.kalman_filter = kalman_filter
        self.analysis_results = AnalysisResults(self.share_dir, self.csv_path, self.name_file)
        self.visualizer = VisualResults()
        self.camera_controller = CameraController()
        self.last_frame = None
        self.real_data = {}
        self.predicted_data = {}
        self.predicted_data_all = {}

    def handle_camera_image(self, msg):

        measurement = self.cv_processor.get_measurement_frame(msg)
        # ініціалізуємо відео на першому кадрі
        if self.pose_writer is None:
            frame = measurement["frame"]
            h, w = frame.shape[:2]
            self.pose_writer = cv2.VideoWriter(self.pose_path, self.fourcc, self.fps, (w, h))

        # Візуалізація центроїду та орієнтації
        vis_pose = self.visualizer.draw_pose(
            frame=measurement["frame"].copy(),
            centroid=measurement["centroid"],
            angle=measurement["angle"],
            contour=measurement["contour"],
            valid = measurement["valid"]
        )
        self.pose_writer.write(vis_pose)
        cv2.imshow(f"Computer Vision Result {self.name_file}", vis_pose)
        cv2.waitKey(1)

        self.last_frame = measurement["frame"]
        predicted = self.kalman_filter.predict(measurement) # Далі Mediator передає оброблені дані в UnscentedFilterKalman
        if predicted:
            time = predicted["time"]
            data_all = predicted["predicted"]
            if data_all is not None:
                self.add_predicted(data_all, time) # Зберігаємо прогнозовані дані для синхронізації

    def add_real(self, msg):  # Реальні дані (з Unity)
        time = round(msg[0], 2)
        self.real_data[time] = msg[1:]

    def add_predicted(self, data_all, time): # Прогнозовані дані (з UnscentedFilterKalman)
        time = round(time, 2)
        self.predicted_data[time] = data_all[0], data_all[1]  # (x, y)
        self.camera_controller.send_target_point([data_all[0], data_all[1]])
        self.predicted_data_all[time] = data_all
        self.sync_points()

    def sync_points(self, tolerance=0.03): # Синхронізація прогнозованих та реальних точок

        #print(f"real_data = {self.real_data.keys()}")
        #print(f"predicted_data = {self.predicted_data.keys()}")

        for predicted_time in list(self.predicted_data.keys()):

            closest_real_time = None
            min_diff = float('inf')

            for real_time in self.real_data.keys():
                diff = abs(real_time - predicted_time)
                if diff <= tolerance and diff < min_diff:
                    min_diff = diff
                    closest_real_time = real_time

            if closest_real_time is not None:

                predicted_point = self.predicted_data[predicted_time]
                real_point = self.real_data[closest_real_time]

                self.analysis_results.compare(
                    predicted_point,
                    real_point,
                    predicted_time
                )

                if self.last_frame is not None:
                    # ініціалізуємо відео на першому кадрі
                    if self.cmp_writer is None:
                        h, w = self.last_frame.shape[:2]
                        self.cmp_writer = cv2.VideoWriter(self.cmp_path, self.fourcc, self.fps, (w, h))

                    vis = self.visualizer.draw(self.last_frame.copy(), real_point, predicted_point)
                    self.cmp_writer.write(vis)
                    cv2.imshow(f"Real vs Predicted {self.name_file}", vis)
                    cv2.waitKey(1)

                row = [predicted_time] + list(real_point) + self.predicted_data_all[predicted_time]
                row = [f"{x:.3f}" for x in row] # Форматуємо кожне значення як рядок з 3 десятковими знаками
                self.csv_writer.writerow(row)
                self.csv_file.flush()  # щоб дані одразу потрапили на диск

                del self.predicted_data[predicted_time]
                del self.predicted_data_all[predicted_time]
                del self.real_data[closest_real_time]

            else:
                pass
                #print(f"⚠️ [Sync] Для прогнозованої точки з часом {predicted_time} не знайдено відповідної реальної точки у межах {tolerance} с.")

    def close_writers(self):
        # звільняє ресурси VideoWriter-ів і закриває вікна
        if self.pose_writer is not None:
            self.pose_writer.release()
        if self.cmp_writer is not None:
            self.cmp_writer.release()
        cv2.destroyAllWindows()


#Mediator приймає сирі повідомлення від Observer.
#Далі Mediator вирішує, кому передати ці дані. Спочатку він надсилає їх у ComputerVision.
#ComputerVision повертає центроїд і орієнтацію назад у Mediator.
#Mediator передає ці оброблені дані в UnscentedFilterKalman.
#UnscentedFilterKalman повертає прогнозовану точку назад до Mediator.
#Mediator синхронізує прогнозовані й реальні точки та передає їх до AnalysisResults.