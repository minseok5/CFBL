import sys
import subprocess
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QTableWidget,
    QTableWidgetItem, QLabel, QPushButton, QHBoxLayout
)
from PyQt5.QtCore import Qt, QTimer


class CrazyflieStatusTable(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Crazyflie Status")
        self.setGeometry(300, 300, 700, 300)

        self.drone_ids = ["cf01", "cf02", "cf03"]
        self.columns = ["Status", "Battery", "x", "y", "z"]

        self.status_data = {drone_id: {"status": "Idle", "battery": "N/A", "x": 0.0, "y": 0.0, "z": 0.0} for drone_id in self.drone_ids}

        layout = QVBoxLayout()
        self.table = QTableWidget(len(self.drone_ids), len(self.columns))
        self.table.setHorizontalHeaderLabels(self.columns)
        self.table.setVerticalHeaderLabels([d.upper() for d in self.drone_ids])
        self.table.verticalHeader().setDefaultAlignment(Qt.AlignCenter)

        for row in range(len(self.drone_ids)):
            for col in range(len(self.columns)):
                item = QTableWidgetItem("")
                item.setTextAlignment(Qt.AlignCenter)
                item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
                self.table.setItem(row, col, item)

        layout.addWidget(QLabel("Status Panel"))
        layout.addWidget(self.table)

        button_layout = QHBoxLayout()
        self.arm_button = QPushButton("Arm")
        self.hover_button = QPushButton("Hover")
        self.launch_button = QPushButton("Launch Crazyflie")

        self.arm_button.clicked.connect(self.run_arm)
        self.hover_button.clicked.connect(self.run_hover)
        self.launch_button.clicked.connect(self.run_launch)

        button_layout.addWidget(self.arm_button)
        button_layout.addWidget(self.hover_button)
        button_layout.addWidget(self.launch_button)
        layout.addLayout(button_layout)

        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_table)
        self.timer.start(1000)

    def refresh_table(self):
        for row, drone_id in enumerate(self.drone_ids):
            data = self.status_data[drone_id]
            self.table.setItem(row, 0, QTableWidgetItem(data["status"]))
            self.table.setItem(row, 1, QTableWidgetItem(str(data["battery"])))
            self.table.setItem(row, 2, QTableWidgetItem(f"{data['x']:.2f}"))
            self.table.setItem(row, 3, QTableWidgetItem(f"{data['y']:.2f}"))
            self.table.setItem(row, 4, QTableWidgetItem(f"{data['z']:.2f}"))

    def update_drone_status(self, drone_id, status=None, battery=None, x=None, y=None, z=None):
        if drone_id not in self.status_data:
            return
        if status is not None:
            self.status_data[drone_id]["status"] = status
        if battery is not None:
            self.status_data[drone_id]["battery"] = battery
        if x is not None:
            self.status_data[drone_id]["x"] = x
        if y is not None:
            self.status_data[drone_id]["y"] = y
        if z is not None:
            self.status_data[drone_id]["z"] = z

    def run_arm(self):
        self.set_all_status("Arming...")
        threading.Thread(target=self.run_ros2_command, args=(["ros2", "run", "cfbl_control", "arm"], "Armed")).start()

    def run_hover(self):
        self.set_all_status("Hovering...")
        threading.Thread(target=self.run_ros2_command, args=(["ros2", "run", "cfbl_control", "hover"], "Hover")).start()

    def run_launch(self):
        threading.Thread(target=self.run_ros2_command, args=(["ros2", "launch", "crazyflie", "launch.py"], None)).start()

    def run_ros2_command(self, cmd, final_status):
        subprocess.run(cmd)
        if final_status:
            self.set_all_status(final_status)

    def set_all_status(self, status):
        for drone_id in self.drone_ids:
            self.status_data[drone_id]["status"] = status


class ROS2Listener(Node):
    def __init__(self, gui: CrazyflieStatusTable):
        super().__init__("crazyflie_gui_listener")
        self.gui = gui

        for drone_id in gui.drone_ids:
            self.create_subscription(Float32, f"/{drone_id}/battery", self.make_battery_callback(drone_id), 10)
            self.create_subscription(PoseStamped, f"/{drone_id}/pose", self.make_pose_callback(drone_id), 10)

    def make_battery_callback(self, drone_id):
        def callback(msg):
            self.gui.update_drone_status(drone_id, battery=f"{msg.data:.2f}V")
        return callback

    def make_pose_callback(self, drone_id):
        def callback(msg):
            p = msg.pose.position
            self.gui.update_drone_status(drone_id, x=p.x, y=p.y, z=p.z)
        return callback


def start_ros2_node(gui: CrazyflieStatusTable):
    rclpy.init()
    node = ROS2Listener(gui)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = CrazyflieStatusTable()
    gui.show()

    threading.Thread(target=start_ros2_node, args=(gui,), daemon=True).start()

    sys.exit(app.exec_())
