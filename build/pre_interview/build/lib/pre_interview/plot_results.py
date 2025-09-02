import csv
import os
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt

class PlotResults(Node):
    def __init__(self):
        super().__init__('plot_results')
        self.declare_parameter('csv_path', 'speed_log.csv')
        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value

    def run(self):
        if not os.path.exists(self.csv_path):
            self.get_logger().error(f"CSV not found: {self.csv_path}")
            return
        t, target, meas, cmd = [], [], [], []
        with open(self.csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for r in reader:
                t.append(float(r['time_s']))
                target.append(float(r['target_kmh']))
                meas.append(float(r['meas_kmh']))
                cmd.append(float(r['cmd']))

        plt.figure()
        plt.plot(t, target, label='Target (km/h)')
        plt.plot(t, meas, label='Actual (km/h)')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (km/h)')
        plt.title('Speed vs Time')
        plt.legend()
        plt.grid(True)

        plt.figure()
        plt.plot(t, cmd, label='u (throttle/brake)')
        plt.xlabel('Time (s)')
        plt.ylabel('Control [-1,1]')
        plt.title('Control Command vs Time')
        plt.legend()
        plt.grid(True)

        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = PlotResults()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

