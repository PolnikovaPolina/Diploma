import os, math, csv, atexit
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

class AnalysisResults:
    def __init__(self, shared_dir = None,  name_file = None, max_step_pred = None):
        self.shared_dir = shared_dir
        self.name_file = name_file

        # CSV таблиця
        self.csv_path = os.path.join(self.shared_dir, f'results_predict_ukf_{self.name_file}.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        atexit.register(self.csv_file.close)  # файл закриється при виході з програми
        self.csv_result = csv.writer(self.csv_file)

        # CSV таблиця траєкторій
        self.max_step_pred = max_step_pred
        self.csv_path_traj = os.path.join(self.shared_dir, f'results_trajectory_{self.name_file}.csv')
        self.csv_file_traj = open(self.csv_path_traj, 'w', newline='')
        atexit.register(self.csv_file_traj.close)  # файл закриється при виході з програми
        self.csv_result_traj = csv.writer(self.csv_file_traj)

        self.error_series = []
        self.dx_series = []
        self.dy_series = []
        self.times = []

        # Файл для CameraController
        self.csv_cam_path = os.path.join(self.shared_dir, f"result_camera_controll_{self.name_file}.csv")
        self.csv_cam_file = open(self.csv_cam_path, "w", newline="")
        atexit.register(self.csv_cam_file.close)
        self.csv_cam_writer = csv.writer(self.csv_cam_file)
        self.csv_cam_writer.writerow(["time", "theta", "theta_gt", "psi", "psi_gt"])  # заголовки
        atexit.register(self.csv_cam_file.close)

    def compare(self, predicted, real, t):
        if real is not None and predicted is not None:
            dx = real[0] - predicted[0]
            dy = real[1] - predicted[1]
            error = (dx ** 2 + dy **  2) ** 0.5
            self.dx_series.append(dx)
            self.dy_series.append(dy)
            self.error_series.append(error)
            self.times.append(t)
            #print(f"Δx={dx:.4f}, Δy={dy:.4f}, L2={error:.4f}")

    def save_plot(self, data, label):
        plt.figure(figsize=(10, 5))
        plt.plot(self.times, data, label=label)
        plt.ylabel(label)
        plt.title("Prediction vs Real Pixel Position Error")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        name_graf = f"prediction_{label}_{self.name_file}.png"
        plot_path = os.path.join(self.shared_dir, name_graf)
        plt.savefig(plot_path)
        print(f"Графік збережено як '{name_graf}'")

    def save_all(self):
        print("Зберігаємо графіки...")
        #self.save_plot(self.dx_series, "dx")
        #self.save_plot(self.dy_series, "dy")
        #self.save_plot(self.error_series, "L2")
        #self.save_from_csv()
        #self.save_analysis()
        self.save_from_camera_csv()

    def errors(self, variable, df):
        mse = []
        for step_pred in range(self.max_step_pred + 1):  # 0...10
            pred_col = f'pred_{variable}_{step_pred}'
            suma_mse = count = 0
            for i, real_x in enumerate(df[f'real_{variable}'][step_pred:], start=step_pred):
                suma_mse += (real_x - df[pred_col].iloc[i - step_pred]) ** 2
                #print(f"{step_pred}: real_{variable} - pred_col = {real_x} - {df[pred_col].iloc[i - step_pred]} = {(real_x - df[pred_col].iloc[i - step_pred]) ** 2}")
                count += 1
            #print(f"count = {count}")
            mse.append(suma_mse / count if count > 0 else float('nan'))
        return mse

    def graf_cv_vs_real_with_noise_and_clean(self, df):
        cv_x = df['cv_x']
        cv_y = df['cv_y']
        real_x = df['real_x']
        real_y = df['real_y']

        # Генеруємо зашумлені дані
        cv_x_noisy = cv_x + np.random.normal(0, 0.1, size=len(df))
        cv_y_noisy = cv_y + np.random.normal(0, 0.1, size=len(df))

        fig2, axs = plt.subplots(2, 2, figsize=(12, 8), sharex=True)

        # Колірна палітра
        color_real = '#f7f584'  # істинне значення
        color_cv = '#46c2eb'  # виміряне значення

        # Лівий стовпчик — зашумлені дані
        axs[0, 0].plot(range(len(real_x)), real_x, color=color_real, marker='o', label='істинне X')
        axs[0, 0].plot(range(len(cv_x_noisy)), cv_x_noisy, color=color_cv, marker='o', label='виміряне X (з шумом)')
        axs[0, 0].set_ylabel("X координата")
        axs[0, 0].set_title("Зашумлені дані (μ=0, σ²=0.1²)")
        axs[0, 0].legend()
        axs[0, 0].grid(True)

        axs[1, 0].plot(range(len(real_y)), real_y, color=color_real, marker='o', label='істинне Y')
        axs[1, 0].plot(range(len(cv_y_noisy)), cv_y_noisy, color=color_cv, marker='o', label='виміряне Y (з шумом)')
        axs[1, 0].set_xlabel("Час")
        axs[1, 0].set_ylabel("Y координата")
        axs[1, 0].legend()
        axs[1, 0].grid(True)

        # Правий стовпчик — чисті дані
        axs[0, 1].plot(range(len(real_x)), real_x, color=color_real, marker='o', markersize=2, label='істинне X')
        axs[0, 1].plot(range(len(cv_x)), cv_x, color=color_cv, marker='o', markersize=2, label='виміряне X')
        axs[0, 1].set_title("Чисті дані")
        axs[0, 1].legend()
        axs[0, 1].grid(True)

        axs[1, 1].plot(range(len(real_y)), real_y, color=color_real, marker='o', markersize=2, label='істинне Y')
        axs[1, 1].plot(range(len(cv_y)), cv_y, color=color_cv, marker='o', markersize=2, label='виміряне Y')
        axs[1, 1].set_xlabel("Час")
        axs[1, 1].legend()
        axs[1, 1].grid(True)

        plt.tight_layout()
        out_path = os.path.join(self.shared_dir, f"cv_vs_real_with_noise_and_clean_{self.name_file}.png")
        plt.savefig(out_path)
        plt.close()
        print(f"Графік cv_vs_real_with_noise_and_clean_{self.name_file}.png збережено")

    def graf_errors(self, mse_x, mse_y):
        steps = list(range(self.max_step_pred + 1))
        fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        # Верхній графік (MSE по X)
        axes[0].plot(steps, mse_x, marker='o', label='MSE по X')
        for i, mse in enumerate(mse_x):
            axes[0].text(i, mse, f"{mse:.3f}", ha='center', va='bottom', fontsize=8)
        axes[0].set_ylabel("MSE (X)")
        axes[0].set_title("Похибка MSE по X для кожного кроку прогнозу")
        axes[0].grid(True)
        axes[0].legend()
        # Нижній графік (MSE по Y)
        axes[1].plot(steps, mse_y, marker='o', color='orange', label='MSE по Y')
        for i, mse in enumerate(mse_y):
            axes[1].text(i, mse, f"{mse:.3f}", ha='center', va='bottom', fontsize=8)
        axes[1].set_xlabel("Крок прогнозу")
        axes[1].set_ylabel("MSE (Y)")
        axes[1].set_title("Похибка MSE по Y для кожного кроку прогнозу")
        axes[1].grid(True)
        axes[1].legend()
        plt.tight_layout()
        out_path = os.path.join(self.shared_dir, f"prediction_{self.name_file}.png")
        plt.savefig(out_path)
        plt.close()
        print(f"Графік prediction_{self.name_file}.png збережено")

    def graf_real_vs_cv_vs_pred0(self, df):
        fig, axs = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
        #cv_x_noisy = df['cv_x'] + np.random.normal(0, 0.01, size=len(df))
        #cv_y_noisy = df['cv_y'] + np.random.normal(0, 0.01, size=len(df))
        #pred_x_noisy = df['pred_x_0'] + np.random.normal(0, 0.0001, size=len(df))
        #pred_y_noisy = df['pred_y_0'] + np.random.normal(0, 0.0001, size=len(df))
        axs[0].plot(df['real_x'], label='істинне X', color='gold', marker='o')
        axs[0].plot(df['cv_x'], label='виміряне X (CV)', color='blue', marker='o')  # буде розрив, якщо є NaN
        axs[0].plot(df['pred_x_0'], label='прогноз X', color='green', linestyle='--')
        axs[0].set_ylabel("X координата")
        axs[0].set_title("X: Істинне vs виміряне vs прогноз")
        axs[0].legend()
        axs[0].grid(True)

        axs[1].plot(df['real_y'], label='істинне Y', color='dodgerblue', marker='o')
        axs[1].plot(df['cv_y'], label='виміряне Y (CV)', color='orange', marker='o')  # теж підтримує NaN
        axs[1].plot(df['pred_y_0'], label='прогноз Y', color='green', linestyle='--', marker='o')
        axs[1].set_xlabel("Час")
        axs[1].set_ylabel("Y координата")
        axs[1].set_title("Y: Істинне vs виміряне vs прогноз")
        axs[1].legend()
        axs[1].grid(True)

        plt.tight_layout()
        out_path = os.path.join(self.shared_dir, f"real_vs_cv_vs_pred0_{self.name_file}.png")
        plt.savefig(out_path)
        plt.close()
        print(f"Графік real_vs_cv_vs_pred0_{self.name_file}.png збережено")

    def save_analysis(self):
        df = pd.read_csv(self.csv_path_traj)
        # побудова графіка прогнозу
        mse_x = self.errors("x", df)
        mse_y = self.errors("y", df)
        self.graf_errors(mse_x, mse_y)
        self.graf_cv_vs_real_with_noise_and_clean(df)
        self.graf_real_vs_cv_vs_pred0(df)

    def save_from_csv(self): # Зчитати дані з CSV і побудувати графіки для всіх колонок, крім 'time'.
        df = pd.read_csv(self.csv_path)

        times = df['time'].tolist()
        metrics_pairs = [('real_x', 'pred_ukf_x'), ('real_y', 'pred_ukf_y')]
        all_possible_metrics = ['pred_ukf_v_x', 'pred_ukf_v_y', 'pred_ukf_a_x', 'pred_ukf_a_y', 'pred_ukf_theta',
                                'pred_ukf_theta_w', 'pred_ukf_theta_a']
        other_metrics = [m for m in all_possible_metrics if m in df.columns]

        nrows = len(metrics_pairs) + len(other_metrics)
        fig, axes = plt.subplots(nrows, 1, figsize=(12, 4 * nrows))

        if nrows == 1:
            axes = [axes]

        # Combined real vs predicted plots
        for ax, (real_metric, pred_metric) in zip(axes, metrics_pairs):
            ax.plot(times, df[real_metric], label=real_metric, color='blue', marker='o')
            ax.plot(times, df[pred_metric], label=pred_metric, color='orange', marker='x')

            ax.set_title(f'{real_metric} vs {pred_metric}')
            ax.set_xlabel('Time')
            ax.set_ylabel('Value')
            ax.grid(True)
            ax.legend()

        # Individual plots for other metrics
        for i, metric in enumerate(other_metrics, start=len(metrics_pairs)):
            ax = axes[i]
            ax.plot(times, df[metric], label=metric, color='green', marker='.')
            ax.set_title(metric)
            ax.set_xlabel('Time')
            ax.set_ylabel('Value')
            ax.grid(True)
            ax.legend()

        plt.tight_layout()
        filename = f"combined_{self.name_file}.png"
        if self.shared_dir:
            os.makedirs(self.shared_dir, exist_ok=True)
            out_path = os.path.join(self.shared_dir, filename)
        else:
            out_path = filename

        fig.savefig(out_path)
        plt.close(fig)
        print(f"Об'єднаний графік збережено як '{self.name_file}'")

    def log_camera_control(self, time, theta, psi, theta_gt, psi_gt):
        row = (
            f"{time:.3f}",
            f"{theta:.3f}", f"{theta_gt:.3f}",
            f"{psi:.3f}", f"{psi_gt:.3f}", )
        self.csv_cam_writer.writerow(row)
        self.csv_cam_file.flush()

    def save_from_camera_csv(self):
        df = pd.read_csv(self.csv_cam_path)
        # Створюємо фігу з двома рядами, спільна вісь X (час)
        fig, ax = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        # ---------- верхній підграфік: θ (elevation) ----------
        ax[0].plot(df["time"], df["theta_gt"], label="θ GT", marker='.')
        ax[0].plot(df["time"], df["theta"], label="θ pred", marker='x')
        ax[0].set_ylabel("θ, rad")
        ax[0].set_title("Elevation angle (θ)")
        ax[0].grid(True);
        ax[0].legend()
        # ---------- нижній підграфік: ψ (azimuth) ----------
        ax[1].plot(df["time"], df["psi_gt"], label="ψ GT", marker='.')
        ax[1].plot(df["time"], df["psi"], label="ψ pred", marker='x')
        ax[1].set_xlabel("Time (s)")
        ax[1].set_ylabel("ψ, rad")
        ax[1].set_title("Azimuth angle (ψ)")
        ax[1].grid(True);
        ax[1].legend()
        out_path = os.path.join(self.shared_dir, f"angles_{self.name_file}.png") if self.shared_dir else f"angles_{self.name_file}.png"
        fig.tight_layout()
        fig.savefig(out_path)
        plt.show()
        plt.close(fig)
        print(f"Графік θ та ψ збережено як '{out_path}'")
