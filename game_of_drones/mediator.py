import os, cv2  # автоматично закриття cvs файлу
import numpy as np

from GameOfDronesDev.game_of_drones.visualization_analysis.analysis_results import AnalysisResults
from GameOfDronesDev.game_of_drones.visualization_analysis.visual_results import VisualResults
from GameOfDronesDev.game_of_drones.camera_controller import CameraController


class Mediator:
    def __init__(self, cv_processor = None, kalman_filter = None, analysis_results = None, step = None, share_dir = None, name_file = None):

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
        # Третє відео — траєкторія:
        self.traj_path = os.path.join(self.share_dir, f'trajectory_{self.name_file}.avi')
        self.traj_writer = None
        self.traj_canvas = None

        self.max_step_pred = 10

        self.cv_processor = cv_processor
        self.kalman_filter = kalman_filter
        self.analysis_results = AnalysisResults(self.share_dir, self.name_file, self.max_step_pred)
        self.visualizer = VisualResults()

        self.step_pred_track_drone = step_pred_track_drone
        self.camera_controller = CameraController(analysis_results = self.analysis_results)
        self.manual_mode = False

        self.last_frame = None
        self.real_data = {}
        self.predicted_data = {}
        self.predicted_data_all = {}

        if "simple" in name_file:
            self.field_names = ["x", "y", "v_x", "v_y", "a_x", "a_y"]
            self.csv_result.writerow([
                'time',
                'dt',
                'real_x', 'real_y',
                'pred_ukf_x', 'pred_ukf_y', 'pred_ukf_v_x', 'pred_ukf_v_y', 'pred_ukf_a_x', 'pred_ukf_a_y'
            ])

        if "full" in name_file:
            self.field_names = ["x", "y", "v_x", "v_y", "a_x", "a_y", "theta", "theta_w", "theta_a"]
            self.csv_result.writerow([
                'time',
                'dt',
                'real_x', 'real_y',
                'pred_ukf_x', 'pred_ukf_y', 'pred_ukf_v_x', 'pred_ukf_v_y', 'pred_ukf_a_x', 'pred_ukf_a_y', 'pred_ukf_theta', 'pred_ukf_theta_w'
            ])

        header = ['time', 'dt','real_x', 'real_y', 'cv_x', 'cv_y',]
        for i in range(self.max_step_pred + 1):
            header.append(f'pred_x_{i}')
            header.append(f'pred_y_{i}')
        self.csv_result_traj.writerow(header)

    def handle_camera_image(self, msg):
        measurement = self.cv_processor.get_measurement_frame(msg)

        #print(f"msg.status.data = {msg.status.data}")

        # автоматично наводимо камеру на ціль
        key = cv2.waitKey(1) & 0xFF
        if key == ord('d'):
            self.manual_mode = not self.manual_mode

        # створення першого кадру на відео та візуалізація центроїду та орієнтації
        self.pose_writer = self.initial_first_frame(self.pose_writer, self.pose_path, measurement["frame"])
        vis_pose = self.visualizer.draw_pose(
            frame=measurement["frame"].copy(),
            centroid=measurement["centroid"],
            angle=measurement["angle"],
            contour=measurement["contour"],
            valid = measurement["valid"])
        self.pose_writer.write(vis_pose)
        cv2.imshow(f"Computer Vision Result {self.name_file}", vis_pose)

        self.last_frame = measurement["frame"]
        #self.camera_controller.track_drone(measurement)
        predicted = self.kalman_filter.predict(measurement) # Далі Mediator передає оброблені дані в UnscentedFilterKalman
        if predicted:
            #if predicted["predicted"] is not None: #? можна позбутися?
            self.add_predicted(predicted, measurement) # Зберігаємо прогнозовані дані для синхронізації
            if self.manual_mode:
                x_pred_step = self.kalman_filter.motion_model.predict_steps(np.asarray(predicted["predicted"]), predicted["dt"], self.step_pred_track_drone)
                #self.camera_controller.track_drone(measurement)

    def add_real(self, msg):  # Реальні дані (з Unity)
        time = round(msg[0], 3)
        self.real_data[time] = msg[1:]

    def add_predicted(self, predicted, measurement): # Прогнозовані дані (з UnscentedFilterKalman)
        data_all = predicted["predicted"]
        time = round(predicted["time"], 3)
        keys = self.field_names
        entry = {key: value for key, value in zip(keys, data_all)}
        entry["dt"] = round(predicted["dt"], 3)
        entry["P"] = predicted["P"]
        entry["Q"] = predicted["Q"]
        # Обробка вимірювання (центроїду з комп’ютерного зору)
        if not measurement["valid"]:
            entry["cv_x"] = float('nan')
            entry["cv_y"] = float('nan')
        else:
            entry["cv_x"] = measurement["centroid"][0]
            entry["cv_y"] = measurement["centroid"][1]

        # Додаємо до загального словника
        self.predicted_data_all[time] = entry
        self.sync_points()

    def sync_points(self, tolerance=0.03): # Синхронізація прогнозованих та реальних точок

        #print(f"real_data = {self.real_data.keys()}")
        #print(f"predicted_data_all = {self.predicted_data_all.keys()}")

        for predicted_time in list(self.predicted_data_all.keys()):
            closest_real_time = None
            min_diff = float('inf')
            for real_time in self.real_data.keys():
                diff = abs(real_time - predicted_time)
                if diff <= tolerance and diff < min_diff:
                    min_diff = diff
                    closest_real_time = real_time

            if closest_real_time is not None:
                predicted_point = (
                    self.predicted_data_all[predicted_time]["x"],
                    self.predicted_data_all[predicted_time]["y"]
                )
                real_point = self.real_data[closest_real_time]

                self.analysis_results.compare(predicted_point, real_point, predicted_time)
                # графіки похибок реальні та вимірянні значення та реальні і відфільтровані

                if self.last_frame is not None:
                    self.cmp_writer = self.initial_first_frame(self.cmp_writer, self.cmp_path, self.last_frame)
                    self.traj_writer = self.initial_first_frame(self.traj_writer, self.traj_path, self.last_frame)
                    if self.traj_canvas is None:  # Створюємо чорне полотно розміром як кадр відео
                        self.traj_canvas = np.zeros_like(self.last_frame)
                    vis = self.visualizer.draw(self.last_frame.copy(), real_point, predicted_point)
                    self.cmp_writer.write(vis)
                    cv2.imshow(f"Real vs Predicted {self.name_file}", vis)

                    canvas = self.traj_canvas
                    vis_traj, pred_trajectory = self.visualizer.draw_trajectory(frame = canvas, real_point = real_point, predicted_data_all = self.predicted_data_all[predicted_time], keys = self.field_names,
                                                               count_pred = 10, method_pred = self.kalman_filter.motion_model.predict_steps_generate_sigma_points,
                                                               P = np.asarray(self.predicted_data_all[predicted_time]["P"]), Q = np.asarray(self.predicted_data_all[predicted_time]["Q"]))
                    self.traj_writer.write(vis_traj)
                    cv2.imshow(f"Trajectory {self.name_file}", vis_traj)

                numeric_keys = [k for k in self.field_names if k in self.predicted_data_all[predicted_time]]
                values = [self.predicted_data_all[predicted_time][k] for k in numeric_keys]
                row = [predicted_time] + [self.predicted_data_all[predicted_time]["dt"]] + list(real_point) + values
                row = [f"{x:.3f}" for x in row] # Форматуємо кожне значення як рядок з 3 десятковими знаками
                self.csv_result.writerow(row)
                self.csv_file.flush()  # щоб дані одразу потрапили на диск

                cv_result = self.predicted_data_all[predicted_time]["cv_x"], self.predicted_data_all[predicted_time]["cv_y"]
                flat_pred_trajectory = [coord for point in pred_trajectory for coord in point]  # розпаковує [(x, y), ...] у [x, y, x, y, ...]
                row_traj = [predicted_time] + [self.predicted_data_all[predicted_time]["dt"]] + list(real_point) + list(cv_result) + flat_pred_trajectory
                self.csv_result_traj.writerow(row_traj)
                self.csv_file_traj.flush()

                del self.predicted_data_all[predicted_time]
                del self.real_data[closest_real_time]

            else:
                pass
                #print(f"⚠️ [Sync] Для прогнозованої точки з часом {predicted_time} не знайдено відповідної реальної точки у межах {tolerance} с.")

    def initial_first_frame(self, writer, path, frame):  # ініціалізуємо відео на першому кадрі
        if writer is None:
            h, w = frame.shape[:2]
            return cv2.VideoWriter(path, self.fourcc, self.fps, (w, h))
        return writer


    def close_writers(self):
        # звільняє ресурси VideoWriter-ів і закриває вікна
        if self.pose_writer is not None:
            self.pose_writer.release()
        if self.cmp_writer is not None:
            self.cmp_writer.release()
        if self.traj_writer:
            self.traj_writer.release()
        cv2.destroyAllWindows()


#Mediator приймає сирі повідомлення від Observer.
#Далі Mediator вирішує, кому передати ці дані. Спочатку він надсилає їх у ComputerVision.
#ComputerVision повертає центроїд і орієнтацію назад у Mediator.
#Mediator передає ці оброблені дані в UnscentedFilterKalman.
#UnscentedFilterKalman повертає прогнозовану точку назад до Mediator.
#Mediator синхронізує прогнозовані й реальні точки та передає їх до AnalysisResults.