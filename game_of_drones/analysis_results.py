import os, math
import matplotlib.pyplot as plt
import pandas as pd

class AnalysisResults:
    def __init__(self, shared_dir, csv_path, name_file):
        self.shared_dir = shared_dir
        self.csv_path = csv_path
        self.name_file = name_file
        self.error_series = []
        self.dx_series = []
        self.dy_series = []
        self.times = []

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
        self.save_plot(self.dx_series, "dx")
        self.save_plot(self.dy_series, "dy")
        self.save_plot(self.error_series, "L2")
        self.save_from_csv()

    def save_from_csv(self): # Зчитати дані з CSV і побудувати графіки для всіх колонок, крім 'time'.
        df = pd.read_csv(self.csv_path)

        times = df['time'].tolist()
        metrics_pairs = [('real_x', 'pred_x'), ('real_y', 'pred_y')]
        other_metrics = ['pred_v_x', 'pred_v_y', 'pred_a_x', 'pred_a_y', 'pred_theta', 'pred_theta_w', 'pred_theta_a']

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