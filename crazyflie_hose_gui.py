import sys
import subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox
)
from PyQt5.QtGui import QFont

class CrazyflieHoseGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Crazyflie GUI')
        self.setGeometry(100, 100, 1000, 800)

        self.cf_id = ["cf01", "cf02", "cf03"]
        self.status_labels = {}
        self.battery_labels = {}
        self.coord_labels = {
            drone_id: {"x": QLabel("0.00"), "y": QLabel("0.00"), "z": QLabel("0.00")}
            for drone_id in self.cf_id
        }

        self.arm_state = False
        self.hover_state = False

        #전체 레이아웃
        main_layout = QVBoxLayout()
        main_layout.addWidget(self._create_status_panel())
        main_layout.addWidget(self._create_control_buttons())
        self.setLayout(main_layout)

       
    def _create_status_panel(self):
        groupbox = QGroupBox("Status")
        groupbox.setFixedHeight(600)
        layout = QGridLayout()

        header_font = QFont()
        header_font.setPointSize(15)
        header_font.setBold(True)

        header_drone = QLabel("Drone")
        header_drone.setFont(header_font)

        header_status = QLabel("Status")
        header_status.setFont(header_font)

        header_battery = QLabel("Battery")
        header_battery.setFont(header_font)

        header_x = QLabel("x")
        header_x.setFont(header_font)

        header_y = QLabel("y")
        header_y.setFont(header_font)

        header_z = QLabel("z")
        header_z.setFont(header_font)

        #Header
        layout.addWidget(header_drone, 0, 0)
        layout.addWidget(header_status, 0, 1)
        layout.addWidget(header_battery, 0, 2)
        layout.addWidget(header_x, 0, 3)
        layout.addWidget(header_y, 0, 4)
        layout.addWidget(header_z, 0, 5)

        # layout.addWidget(QLabel(""))

        for i, cf_id in enumerate(self.cf_id, start=1):
            row = i + 1
            layout.addWidget(QLabel(cf_id.upper()),row, 0)

            status_label = QLabel("Idle")
            layout.addWidget(status_label, row, 1)
            self.status_labels[cf_id] = status_label

            battery_label = QLabel("N/A")
            layout.addWidget(battery_label, row, 2)
            self.battery_labels[cf_id] = battery_label

            layout.addWidget(self.coord_labels[cf_id]["x"], row, 3)
            layout.addWidget(self.coord_labels[cf_id]["y"], row, 4)
            layout.addWidget(self.coord_labels[cf_id]["z"], row, 5)

        groupbox.setLayout(layout)
        return(groupbox)
    
    def _create_control_buttons(self):
        groupbox = QGroupBox("Control")
        layout = QHBoxLayout()

        self.arm_button = QPushButton("Arm")
        self.arm_button.setFixedHeight(40)
        self.arm_button.clicked.connect(self.toggle_arm)
        layout.addWidget(self.arm_button)

        self.hover_button = QPushButton("Hover")
        self.hover_button.setFixedHeight(40)
        self.hover_button.clicked.connect(self.toggle_hover)
        layout.addWidget(self.hover_button)

        self.traj_button = QPushButton("traj")
        self.traj_button.setFixedHeight(40)
        self.traj_button.clicked.connect(self.run_traj)
        layout.addWidget(self.traj_button)

        groupbox.setLayout(layout)
        return groupbox

    def update_position(self, drone_id, x, y, z):
        if drone_id in self.coord_labels:
            self.coord_labels[drone_id]["x"].setText(f"{x: .2f}")
            self.coord_labels[drone_id]["y"].setText(f"{x: .2f}")
            self.coord_labels[drone_id]["z"].setText(f"{x: .2f}")


    def toggle_arm(self):
        cmd = ["ros2", "run", "cfbl_control", "arm"] if self.arm_state else ["ros2", "run", "my_pkg", "arm_node"]
        subprocess.run(cmd)

        self.arm_state = not self.arm_state
        new_status = "Armed" if self.arm_state else "Disarmed"
        self.arm_button.setText("Disarm" if self.arm_state else "Arm")

        for drone_id in self.cf_id:
            self.status_labels[drone_id].setText(new_status)


    def toggle_hover(self):
        cmd = ["ros2", "run", "cfbl_control", "hover"] if self.hover_state else ["ros2", "run", "my_pkg", "hover_node"]
        subprocess.run(cmd)

        self.hover_state = not self.hover_state
        new_status = "Hovering" if self.hover_state else "Landed"
        self.hover_button.setText("Land" if self.hover_state else "Hover")

        for drone_id in self.cf_id:
            self.status_labels[drone_id].setText(new_status)

    def run_traj(self):
        subprocess.run(["ros2", "run", "my_pkg", "traj_node"])

        for drone_id in self.cf_id:
            self.status_labels[drone_id].setText("traj")

    # 추후 ROS2 배터리 수신 토픽에서 호출 예정
    def update_battery_voltage(self, drone_id, voltage):
        if drone_id in self.battery_labels:
            self.battery_labels[drone_id].setText(f"{voltage:.2f}V")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CrazyflieHoseGUI()
    window.show()
    sys.exit(app.exec_())
