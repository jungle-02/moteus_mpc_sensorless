import sys, os, glob, subprocess, zmq, json, collections, re, math
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QLineEdit,
    QGroupBox,
    QFormLayout,
    QTextEdit,
    QComboBox,
    QSizePolicy,
    QTabWidget,
    QCheckBox,
    QInputDialog,
)
from PyQt5.QtCore import QTimer, Qt, QProcess


class ControllerGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        # CAU HINH CUA SO
        self.setWindowTitle("Control GUI")
        self.setGeometry(0, 0, 1250, 800)
        self.setMinimumSize(1000, 600)

        # KET NOI ZMQ
        self.context = zmq.Context()
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect("tcp://localhost:5556")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.connect("tcp://localhost:5557")

        # bien trang thai ket noi
        self.is_connected = False
        self.last_heartbeat = 0
        self.connect_request_time = None
        self.waiting_for_connection = False
        self.current_mode = "Normal"
        self.connection_start_time = None
        self.last_motor_error_code = 0
        self.pid_dq_request_sent = False

        # luu log calib
        self.calib_process = None
        self.calib_output_text = ""

        # KHOI TAO DU LIEU DO THI
        self.max_points = 10000
        self.times = collections.deque(maxlen=self.max_points)

        # du lieu tab 1
        self.cmd_pos = collections.deque(maxlen=self.max_points)
        self.act_pos = collections.deque(maxlen=self.max_points)
        self.sl_pos = collections.deque(maxlen=self.max_points)

        self.cmd_vel = collections.deque(maxlen=self.max_points)
        self.act_vel = collections.deque(maxlen=self.max_points)
        self.sl_vel = collections.deque(maxlen=self.max_points)

        # du lieu tab 2
        self.err_pos = collections.deque(maxlen=self.max_points)
        self.err_vel = collections.deque(maxlen=self.max_points)
        self.var_pos_list = collections.deque(maxlen=self.max_points)
        self.var_vel_list = collections.deque(maxlen=self.max_points)

        self.err_as_pos = collections.deque(
            maxlen=self.max_points
        )  # thuc te tru sensorless
        self.err_sc_pos = collections.deque(
            maxlen=self.max_points
        )  # sensorless tru cmd
        self.var_as_pos = collections.deque(maxlen=self.max_points)
        self.var_sc_pos = collections.deque(maxlen=self.max_points)

        self.err_as_vel = collections.deque(maxlen=self.max_points)
        self.err_sc_vel = collections.deque(maxlen=self.max_points)
        self.var_as_vel = collections.deque(maxlen=self.max_points)
        self.var_sc_vel = collections.deque(maxlen=self.max_points)

        # du lieu tab 3
        self.act_torque = collections.deque(maxlen=self.max_points)
        self.cmd_torque = collections.deque(maxlen=self.max_points)
        self.iq_values = collections.deque(maxlen=self.max_points)
        self.id_values = collections.deque(maxlen=self.max_points)

        # du lieu tab 4
        self.r_values = collections.deque(maxlen=self.max_points)
        self.phi_values = collections.deque(maxlen=self.max_points)
        self.l_values = collections.deque(maxlen=self.max_points)
        self.latest_temp = 0.0

        self.current_script_process = None
        self.scripts_folder = r"D:\moteus\test\scripts"

        # DUONG DAN TUYET DOI
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.calib_file_path = os.path.join(current_dir, "calib_data.json")

        if not os.path.exists(self.scripts_folder):
            try:
                os.makedirs(self.scripts_folder)
                self.log_to_console(f"Created script directory: {self.scripts_folder}")
            except Exception as e:
                print(f"Error creating script directory: {e}")

        self.legend_callbacks = []
        self.chart_lines = {}

        self.init_ui()

        # timer cap nhat do thi
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data_and_plot)
        self.timer.start(20)

        # TU DONG KHOI CHAY BACKEND
        self.backend_process = None
        self.start_backend()

    def clear_plot_data(self):
        self.times.clear()
        self.cmd_pos.clear()
        self.act_pos.clear()
        self.sl_pos.clear()
        self.cmd_vel.clear()
        self.act_vel.clear()
        self.sl_vel.clear()
        self.err_pos.clear()
        self.err_vel.clear()
        self.var_pos_list.clear()
        self.var_vel_list.clear()
        self.err_as_pos.clear()
        self.err_sc_pos.clear()
        self.var_as_pos.clear()
        self.var_sc_pos.clear()
        self.err_as_vel.clear()
        self.err_sc_vel.clear()
        self.var_as_vel.clear()
        self.var_sc_vel.clear()
        self.act_torque.clear()
        self.cmd_torque.clear()
        self.iq_values.clear()
        self.id_values.clear()
        self.r_values.clear()
        self.phi_values.clear()
        self.l_values.clear()

    def start_backend(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        backend_path = os.path.join(current_dir, "backend.py")

        if os.path.exists(backend_path):
            self.log_to_console("Started backend process")
            self.backend_process = subprocess.Popen([sys.executable, backend_path])
        else:
            self.log_to_console(
                f"ERROR: File not found {backend_path}. Please ensure it's in the same directory!",
                is_cmd=True,
            )

    def closeEvent(self, event):
        self.log_to_console("Cleaning up system...", is_cmd=True)
        self.timer.stop()
        self.stop_script()

        if self.calib_process:
            self.calib_process.kill()

        if self.backend_process:
            try:
                self.backend_process.terminate()
                self.backend_process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.backend_process.kill()
                self.log_to_console(
                    "Backend process forcefully terminated.", is_cmd=True
                )

        try:
            self.sub_socket.setsockopt(zmq.LINGER, 0)
            self.pub_socket.setsockopt(zmq.LINGER, 0)
            self.sub_socket.close()
            self.pub_socket.close()
            self.context.term()
        except Exception as e:
            print(f"Error closing ZMQ: {e}")

        print("System has been cleaned up.")
        event.accept()

    def create_dash_label(self, text, color):
        lbl = QLabel(text)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet(f"""
            QLabel {{
                background-color: #fafafa;
                border: 1px solid {color};
                border-radius: 4px;
                padding: 4px 10px;
                font-size: 14px;
                font-weight: bold;
                color: {color};
            }}
        """)
        lbl.setMinimumHeight(30)
        lbl.setMaximumHeight(40)
        return lbl

    def setup_interactive_legend(self, ax, canvas, extra_lines=None):
        lines = list(ax.get_lines())
        if extra_lines:
            lines.extend(extra_lines)
        if not lines:
            return
        if ax.get_legend() is not None:
            ax.get_legend().remove()

        labels = [l.get_label() for l in lines]
        leg = ax.legend(lines, labels, loc="upper right")
        map_legend_to_ax = {}

        for legline, legtext, origline in zip(leg.get_lines(), leg.get_texts(), lines):
            legline.set_picker(True)
            legline.set_pickradius(10)
            legtext.set_picker(True)
            map_legend_to_ax[legline] = origline
            map_legend_to_ax[legtext] = origline

        def on_pick(event):
            leg_artist = event.artist
            origline = map_legend_to_ax.get(leg_artist)
            if origline is None:
                return
            vis = not origline.get_visible()
            origline.set_visible(vis)
            new_alpha = 1.0 if vis else 0.2
            for artist, mapped_line in map_legend_to_ax.items():
                if mapped_line == origline:
                    artist.set_alpha(new_alpha)
            canvas.draw()

        self.legend_callbacks.append(on_pick)
        canvas.mpl_connect("pick_event", on_pick)

    def on_canvas_click(self, event):
        if not event.inaxes:
            return

        if getattr(event, "dblclick", False) or event.button == 3:
            return

        ax = event.inaxes
        x, y = event.xdata, event.ydata

        if hasattr(ax, "click_annot") and ax.click_annot:
            ax.click_annot.remove()
            ax.click_annot = None
            event.canvas.draw_idle()
            return

        text = f"X: {x:.3f}\nY: {y:.3f}"

        ax.click_annot = ax.annotate(
            text,
            xy=(x, y),
            xytext=(15, 15),
            textcoords="offset points",
            bbox=dict(boxstyle="round,pad=0.3", fc="yellow", alpha=0.8),
            arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=0"),
        )
        event.canvas.draw_idle()

    def add_toolbar_and_click(self, canvas, layout, ax=None):
        toolbar = NavigationToolbar(canvas, self)
        toolbar.setMaximumHeight(35)

        top_row = QHBoxLayout()
        top_row.addWidget(toolbar)

        # nut tao duong ke ngang
        btn_line = QPushButton("—")
        btn_line.setToolTip("Add horizontal dashed line")
        btn_line.setFixedSize(28, 28)
        top_row.addWidget(btn_line)
        top_row.addStretch()

        layout.addLayout(top_row)
        layout.addWidget(canvas, stretch=10)

        canvas.toolbar = toolbar

        canvas.mpl_connect("button_press_event", self.on_canvas_click)

        # ket noi nut voi dialog
        if ax is not None:
            btn_line.clicked.connect(
                lambda _, a=ax, c=canvas: self.show_add_line_dialog(a, c)
            )
            # xu ly su kien click
            if not hasattr(canvas, "_line_picker_connected"):
                canvas.mpl_connect("pick_event", self.on_line_pick)
                canvas._line_picker_connected = True

    def show_add_line_dialog(self, ax, canvas):
        try:
            # gia tri mac dinh o giua
            ylim = ax.get_ylim()
            default = (ylim[0] + ylim[1]) / 2.0
            val, ok = QInputDialog.getDouble(
                self, "Add horizontal line", "Y value:", default, -1e12, 1e12, 6
            )
            if ok:
                self.add_hline(ax, canvas, float(val))
        except Exception as e:
            self.log_to_console(f"Error showing input dialog: {e}")

    def add_hline(self, ax, canvas, y):
        try:
            line = ax.axhline(
                y=y, color="black", linestyle="--", linewidth=1.5, picker=15
            )
            lst = self.chart_lines.setdefault(ax, [])
            lst.append(line)
            canvas.draw_idle()
            self.log_to_console(f"Added dashed line at y={y}")
        except Exception as e:
            self.log_to_console(f"Error adding horizontal line: {e}")

    def on_line_pick(self, event):
        try:
            mouse_event = getattr(event, "mouseevent", None)
            if mouse_event is None:
                return

            # xoa duong ke
            is_double_click = getattr(mouse_event, "dblclick", False)
            is_right_click = mouse_event.button == 3

            if not (is_double_click or is_right_click):
                return

            artist = event.artist
            if artist is None:
                return

            ax = artist.axes
            # chi xu ly cac duong duoc tao
            if ax in self.chart_lines and artist in self.chart_lines[ax]:
                try:
                    artist.remove()  # xoa triet de
                except Exception:
                    pass

                try:
                    self.chart_lines[ax].remove(artist)
                except Exception:
                    pass

                if artist.figure:
                    artist.figure.canvas.draw_idle()
                self.log_to_console("Removed dashed line")
        except Exception as e:
            print(f"on_line_pick error: {e}")

    def clear_all_dashed_lines(self):
        """XOA TOAN BO DUONG KE NGANG"""
        for ax, lines in self.chart_lines.items():
            for line in lines:
                try:
                    line.remove()
                except Exception:
                    pass
            # cap nhat giao dien
            if ax.figure:
                ax.figure.canvas.draw_idle()

        # xoa tu dien
        self.chart_lines.clear()

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        # COT TRAI GUI
        left_panel = QVBoxLayout()

        # NHOM KET NOI
        conn_group = QGroupBox("FDCAN Connection")
        conn_layout = QHBoxLayout()
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setStyleSheet(
            "background-color: #0275d8; color: white; font-weight: bold;"
        )
        self.btn_connect.clicked.connect(self.connect_fdcan)

        self.btn_calib = QPushButton("Calibrate")
        self.btn_calib.setStyleSheet(
            "background-color: #f0ad4e; color: black; font-weight: bold;"
        )
        self.btn_calib.clicked.connect(self.run_calibration)
        self.btn_calib.setEnabled(False)

        self.lbl_connection = QLabel("Status: Not connected")
        self.lbl_connection.setStyleSheet("color: red; font-weight: bold;")

        conn_layout.addWidget(self.btn_connect)
        conn_layout.addWidget(self.btn_calib)
        conn_layout.addWidget(self.lbl_connection)
        conn_layout.addStretch()
        conn_group.setLayout(conn_layout)

        # NHOM DIEU KHIEN
        cmd_group = QGroupBox("Control Panel")
        cmd_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        cmd_layout = QFormLayout()

        self.pos_input = QLineEdit()
        self.pos_input.setPlaceholderText("e.g: nan")
        self.vel_input = QLineEdit()
        self.vel_input.setPlaceholderText("e.g: 0.3")
        self.torque_input = QLineEdit()
        self.torque_input.setPlaceholderText("e.g: 0.3")

        self.opts_input = QLineEdit()
        self.opts_input.setPlaceholderText("e.g: p1.0 f0.1 v10")
        self.opts_input.setToolTip(
            "p: kp_scale, d: kd_scale, i: ki_ilimit, s: stop_pos, f: feedforward_Nm\n"
            "t: timeout, v: vel_limit, a: accel_limit, o: fixed_volt, c: fixed_cur, b: ignore_bounds"
        )

        self.btn_send_pos = QPushButton("Send Command")
        self.btn_send_pos.clicked.connect(self.send_d_pos)

        self.mode_selector = QComboBox()
        self.mode_selector.addItems(["Normal", "Sensorless"])
        self.mode_selector.setStyleSheet("""
            QComboBox { padding: 5px; font-weight: bold; border: 1px solid #ccc; border-radius: 4px; }
            QComboBox::drop-down { border: 0px; }
        """)
        self.mode_selector.currentIndexChanged.connect(self.change_mode)

        self.btn_stop = QPushButton("EMERGENCY STOP")
        self.btn_stop.setStyleSheet(
            "background-color: #d9534f; color: white; font-weight: bold; padding: 10px;"
        )
        self.btn_stop.clicked.connect(self.send_d_stop)

        self.kp_d_input = QLineEdit()
        self.kp_d_input.setPlaceholderText("PID DQ Kp")
        self.ki_d_input = QLineEdit()
        self.ki_d_input.setPlaceholderText("PID DQ Ki")
        self.btn_set_pid_dq = QPushButton("Set PID DQ")
        self.btn_set_pid_dq.setStyleSheet(
            "background-color: #5bc0de; color: black; font-weight: bold;"
        )
        self.btn_set_pid_dq.clicked.connect(self.send_pid_dq)

        cmd_layout.addRow("Operation Mode:", self.mode_selector)
        cmd_layout.addRow("Position (rad):", self.pos_input)
        cmd_layout.addRow("Velocity (rad/s):", self.vel_input)
        cmd_layout.addRow("Max Torque (Nm):", self.torque_input)
        cmd_layout.addRow("Optional Args:", self.opts_input)
        cmd_layout.addRow("", self.btn_send_pos)
        cmd_layout.addRow("", self.btn_stop)
        cmd_layout.addRow("PID DQ Kp:", self.kp_d_input)
        cmd_layout.addRow("PID DQ Ki:", self.ki_d_input)
        cmd_layout.addRow("", self.btn_set_pid_dq)
        cmd_group.setLayout(cmd_layout)

        # QUAN LY SCRIPT
        script_group = QGroupBox("Script Manager")
        script_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        script_layout = QVBoxLayout()
        cb_layout = QHBoxLayout()
        self.script_combo = QComboBox()
        self.script_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.refresh_scripts_list()
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self.refresh_scripts_list)
        cb_layout.addWidget(self.script_combo)
        cb_layout.addWidget(self.btn_refresh)

        action_layout = QHBoxLayout()
        self.btn_run_script = QPushButton("Load and Run")
        self.btn_run_script.setStyleSheet(
            "background-color: #5cb85c; color: white; font-weight: bold;"
        )
        self.btn_run_script.setEnabled(False)
        self.btn_run_script.clicked.connect(self.run_selected_script)
        btn_stop_script = QPushButton("Stop Script")
        btn_stop_script.clicked.connect(self.stop_script)
        action_layout.addWidget(self.btn_run_script)
        action_layout.addWidget(btn_stop_script)

        self.lbl_script_status = QLabel("Status: Idle")
        self.lbl_script_status.setStyleSheet("color: gray;")

        script_layout.addLayout(cb_layout)
        script_layout.addLayout(action_layout)
        script_layout.addWidget(self.lbl_script_status)
        script_group.setLayout(script_layout)

        # TERMINAL
        console_group = QGroupBox("Terminal")
        console_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        console_layout = QVBoxLayout()
        self.console_output = QTextEdit()
        self.console_output.setReadOnly(True)
        self.console_output.setStyleSheet(
            "background-color: #1e1e1e; color: #00ff00; font-family: Consolas;"
        )
        self.console_output.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        input_layout = QHBoxLayout()
        self.console_input = QLineEdit()
        self.console_input.setPlaceholderText("Enter command here and press Enter...")
        self.console_input.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.console_input.returnPressed.connect(self.send_console_command)

        btn_clear_console = QPushButton("Clear console")
        btn_clear_console.setStyleSheet("background-color: #f0ad4e; font-weight: bold;")
        btn_clear_console.clicked.connect(self.clear_console)

        input_layout.addWidget(self.console_input)
        input_layout.addWidget(btn_clear_console)
        console_layout.addWidget(self.console_output)
        console_layout.addLayout(input_layout)
        console_group.setLayout(console_layout)

        left_panel.addWidget(conn_group)
        left_panel.addWidget(cmd_group)
        left_panel.addWidget(script_group)
        left_panel.addWidget(console_group)

        # COT PHAI DO THI
        right_panel = QVBoxLayout()
        right_top_bar = QHBoxLayout()

        self.chk_autoscroll = QCheckBox("Auto-Scroll / Auto-Scale")
        self.chk_autoscroll.setChecked(True)
        self.chk_autoscroll.setStyleSheet(
            "font-weight: bold; color: #d9534f; margin-right: 15px;"
        )
        right_top_bar.addWidget(self.chk_autoscroll)

        right_top_bar.addStretch()

        self.tabs = QTabWidget()
        self.tabs.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # TAB 1
        tab1_widget = QWidget()
        tab1_layout = QVBoxLayout(tab1_widget)

        self.fig_pos, self.ax_pos = plt.subplots(figsize=(8, 3))
        self.canvas_pos = FigureCanvas(self.fig_pos)
        self.ax_pos.set_title(
            "Position Control Overview", fontsize=12, fontweight="bold"
        )
        self.ax_pos.set_ylabel("Position (rad)")
        self.ax_pos.grid(True)
        (self.line_cmd_pos,) = self.ax_pos.plot([], [], "b-", label="Command")
        (self.line_act_pos,) = self.ax_pos.plot([], [], "r-", label="Actual (Encoder)")
        (self.line_sl_pos,) = self.ax_pos.plot(
            [],
            [],
            color="darkorange",
            linestyle="-",
            linewidth=1.5,
            label="Sensorless Pos",
        )
        self.fig_pos.tight_layout()
        self.setup_interactive_legend(self.ax_pos, self.canvas_pos)
        self.add_toolbar_and_click(self.canvas_pos, tab1_layout, self.ax_pos)

        pos_info_layout = QHBoxLayout()
        self.lbl_cmd_pos = self.create_dash_label("Cmd: 0.000", "#0000FF")
        self.lbl_act_pos = self.create_dash_label("Act: 0.000", "#FF0000")
        self.lbl_sl_pos = self.create_dash_label("SL: 0.000", "#FF8C00")
        pos_info_layout.addWidget(self.lbl_cmd_pos, 1)
        pos_info_layout.addWidget(self.lbl_act_pos, 1)
        pos_info_layout.addWidget(self.lbl_sl_pos, 1)
        tab1_layout.addLayout(pos_info_layout, stretch=1)

        self.fig_vel, self.ax_vel = plt.subplots(figsize=(8, 3))
        self.canvas_vel = FigureCanvas(self.fig_vel)
        self.ax_vel.set_title(
            "Velocity Control Overview", fontsize=12, fontweight="bold"
        )
        self.ax_vel.set_xlabel("Time (s)")
        self.ax_vel.set_ylabel("Velocity (rad/s)")
        self.ax_vel.grid(True)
        (self.line_cmd_vel,) = self.ax_vel.plot([], [], "m-", label="Command")
        (self.line_act_vel,) = self.ax_vel.plot([], [], "g-", label="Actual (Encoder)")
        (self.line_sl_vel,) = self.ax_vel.plot(
            [], [], color="c", linestyle="-", linewidth=1.5, label="Sensorless Vel"
        )
        self.fig_vel.tight_layout()
        self.setup_interactive_legend(self.ax_vel, self.canvas_vel)
        self.add_toolbar_and_click(self.canvas_vel, tab1_layout, self.ax_vel)

        vel_info_layout = QHBoxLayout()
        self.lbl_cmd_vel = self.create_dash_label("Cmd: 0.000", "#800080")
        self.lbl_act_vel = self.create_dash_label("Act: 0.000", "#008000")
        self.lbl_sl_vel = self.create_dash_label("SL: 0.000", "#00AAAA")
        vel_info_layout.addWidget(self.lbl_cmd_vel, 1)
        vel_info_layout.addWidget(self.lbl_act_vel, 1)
        vel_info_layout.addWidget(self.lbl_sl_vel, 1)
        tab1_layout.addLayout(vel_info_layout, stretch=1)

        self.tabs.addTab(tab1_widget, "Control States")

        # TAB 2
        tab2_widget = QWidget()
        tab2_layout = QVBoxLayout(tab2_widget)

        self.fig_err_pos, self.ax_err_pos = plt.subplots(figsize=(8, 3))
        self.canvas_err_pos = FigureCanvas(self.fig_err_pos)
        self.ax_err_pos.set_title(
            "Position Tracking Error", fontsize=12, fontweight="bold"
        )
        self.ax_err_pos.set_ylabel("Position Error (rad)")
        self.ax_err_pos.grid(True)
        # loi chinh
        (self.line_err_pos_as,) = self.ax_err_pos.plot(
            [], [], "r-", linewidth=1.5, label="Act - SL"
        )
        (self.line_err_pos_sc,) = self.ax_err_pos.plot(
            [], [], color="purple", linewidth=1.0, linestyle="-", label="SL - Cmd"
        )

        self.ax_var_pos = self.ax_err_pos.twinx()
        self.ax_var_pos.set_ylabel("Variance (rad²)")
        (self.line_var_pos_as,) = self.ax_var_pos.plot(
            [],
            [],
            color="darkorange",
            linewidth=1.5,
            linestyle="--",
            label="Var(Act-SL)",
        )
        (self.line_var_pos_sc,) = self.ax_var_pos.plot(
            [], [], color="green", linewidth=1.0, linestyle="--", label="Var(SL-Cmd)"
        )
        self.ax_err_pos.set_zorder(self.ax_var_pos.get_zorder() + 1)
        self.ax_err_pos.patch.set_visible(False)

        self.fig_err_pos.tight_layout()
        self.setup_interactive_legend(
            self.ax_err_pos,
            self.canvas_err_pos,
            extra_lines=[self.line_var_pos_as, self.line_var_pos_sc],
        )
        self.add_toolbar_and_click(self.canvas_err_pos, tab2_layout, self.ax_err_pos)

        err_pos_info_layout = QHBoxLayout()
        self.lbl_err_pos_as = self.create_dash_label("Act-SL: 0.000 rad", "#FF0000")
        self.lbl_err_pos_sc = self.create_dash_label("SL-Cmd: 0.000 rad", "#800080")
        self.lbl_var_pos_as = self.create_dash_label("Var(Act-SL): 0.00e+00", "#8B0000")
        self.lbl_var_pos_sc = self.create_dash_label("Var(SL-Cmd): 0.00e+00", "#008000")
        err_pos_info_layout.addWidget(self.lbl_err_pos_as, 1)
        err_pos_info_layout.addWidget(self.lbl_err_pos_sc, 1)
        err_pos_info_layout.addWidget(self.lbl_var_pos_as, 1)
        err_pos_info_layout.addWidget(self.lbl_var_pos_sc, 1)
        tab2_layout.addLayout(err_pos_info_layout, stretch=1)

        self.fig_err_vel, self.ax_err_vel = plt.subplots(figsize=(8, 3))
        self.canvas_err_vel = FigureCanvas(self.fig_err_vel)
        self.ax_err_vel.set_title(
            "Velocity Tracking Error", fontsize=12, fontweight="bold"
        )
        self.ax_err_vel.set_xlabel("Time (s)")
        self.ax_err_vel.set_ylabel("Velocity Error (rad/s)")
        self.ax_err_vel.grid(True)
        (self.line_err_vel_as,) = self.ax_err_vel.plot(
            [], [], "b-", linewidth=1.5, label="Act - SL"
        )
        (self.line_err_vel_sc,) = self.ax_err_vel.plot(
            [], [], color="darkcyan", linewidth=1.0, linestyle="-", label="SL - Cmd"
        )

        self.ax_var_vel = self.ax_err_vel.twinx()
        self.ax_var_vel.set_ylabel("Variance ((rad/s)²)")
        (self.line_var_vel_as,) = self.ax_var_vel.plot(
            [], [], color="c", linewidth=1.5, linestyle="--", label="Var(Act-SL)"
        )
        (self.line_var_vel_sc,) = self.ax_var_vel.plot(
            [], [], color="orange", linewidth=1.0, linestyle="--", label="Var(SL-Cmd)"
        )
        self.ax_err_vel.set_zorder(self.ax_var_vel.get_zorder() + 1)
        self.ax_err_vel.patch.set_visible(False)

        self.fig_err_vel.tight_layout()
        self.setup_interactive_legend(
            self.ax_err_vel,
            self.canvas_err_vel,
            extra_lines=[self.line_var_vel_as, self.line_var_vel_sc],
        )
        self.add_toolbar_and_click(self.canvas_err_vel, tab2_layout, self.ax_err_vel)

        err_vel_info_layout = QHBoxLayout()
        self.lbl_err_vel_as = self.create_dash_label("Act-SL: 0.000 rad/s", "#0000FF")
        self.lbl_err_vel_sc = self.create_dash_label("SL-Cmd: 0.000 rad/s", "#006064")
        self.lbl_var_vel_as = self.create_dash_label("Var(Act-SL): 0.00e+00", "#00008B")
        self.lbl_var_vel_sc = self.create_dash_label("Var(SL-Cmd): 0.00e+00", "#FF8C00")
        err_vel_info_layout.addWidget(self.lbl_err_vel_as, 1)
        err_vel_info_layout.addWidget(self.lbl_err_vel_sc, 1)
        err_vel_info_layout.addWidget(self.lbl_var_vel_as, 1)
        err_vel_info_layout.addWidget(self.lbl_var_vel_sc, 1)
        tab2_layout.addLayout(err_vel_info_layout, stretch=1)

        self.tabs.addTab(tab2_widget, "Tracking Errors")

        # TAB 3
        tab3_widget = QWidget()
        tab3_layout = QVBoxLayout(tab3_widget)

        self.fig_torque, self.ax_torque = plt.subplots(figsize=(8, 3))
        self.canvas_torque = FigureCanvas(self.fig_torque)
        self.ax_torque.set_title(
            "Command Torque vs Actual Torque", fontsize=12, fontweight="bold"
        )
        self.ax_torque.set_ylabel("Torque (Nm)")
        self.ax_torque.grid(True)
        (self.line_act_torque,) = self.ax_torque.plot(
            [], [], "r-", linewidth=1.5, label="Actual Torque"
        )
        (self.line_cmd_torque,) = self.ax_torque.plot(
            [], [], "b-", linewidth=1.5, label="Command Torque"
        )
        self.fig_torque.tight_layout()
        self.setup_interactive_legend(self.ax_torque, self.canvas_torque)
        self.add_toolbar_and_click(self.canvas_torque, tab3_layout, self.ax_torque)

        torque_info_layout = QHBoxLayout()
        self.lbl_act_torque_val = self.create_dash_label("Actual: 0.000 Nm", "#FF0000")
        self.lbl_cmd_torque_val = self.create_dash_label("Command: 0.000 Nm", "#0000FF")
        torque_info_layout.addWidget(self.lbl_act_torque_val, 1)
        torque_info_layout.addSpacing(10)
        torque_info_layout.addWidget(self.lbl_cmd_torque_val, 1)
        tab3_layout.addLayout(torque_info_layout, stretch=1)

        self.fig_curr, self.ax_curr = plt.subplots(figsize=(8, 3))
        self.canvas_curr = FigureCanvas(self.fig_curr)
        self.ax_curr.set_title(
            "Motor Phase Currents (d-q axis)", fontsize=12, fontweight="bold"
        )
        self.ax_curr.set_xlabel("Time (s)")
        self.ax_curr.set_ylabel("Current (A)")
        self.ax_curr.grid(True)
        (self.line_iq,) = self.ax_curr.plot(
            [], [], "m-", linewidth=1.5, label="q-axis Current (i_q)"
        )
        (self.line_id,) = self.ax_curr.plot(
            [], [], "g-", linewidth=1.5, label="d-axis Current (i_d)"
        )
        self.fig_curr.tight_layout()
        self.setup_interactive_legend(self.ax_curr, self.canvas_curr)
        self.add_toolbar_and_click(self.canvas_curr, tab3_layout, self.ax_curr)

        curr_info_layout = QHBoxLayout()
        self.lbl_iq_val = self.create_dash_label("i_q: 0.000 A", "#800080")
        self.lbl_id_val = self.create_dash_label("i_d: 0.000 A", "#008000")
        curr_info_layout.addWidget(self.lbl_iq_val, 1)
        curr_info_layout.addSpacing(10)
        curr_info_layout.addWidget(self.lbl_id_val, 1)
        tab3_layout.addLayout(curr_info_layout, stretch=1)

        self.tabs.addTab(tab3_widget, "Torque & Current")

        # TAB 4
        tab4_widget = QWidget()
        tab4_layout = QVBoxLayout(tab4_widget)

        self.fig_param, (self.ax_r, self.ax_phi, self.ax_l) = plt.subplots(
            3, 1, figsize=(8, 8), sharex=True
        )
        self.canvas_param = FigureCanvas(self.fig_param)

        self.ax_r.set_title(
            "Stator Resistance Estimation", fontsize=10, fontweight="bold"
        )
        self.ax_r.set_ylabel("R_hat (Ohm)")
        self.ax_r.grid(True)
        (self.line_r,) = self.ax_r.plot([], [], "g-", linewidth=2, label="Estimated R")
        self.setup_interactive_legend(self.ax_r, self.canvas_param)

        self.ax_phi.set_title(
            "Magnetic Flux Estimation", fontsize=10, fontweight="bold"
        )
        self.ax_phi.set_ylabel("Phi_m (mV.s)")
        self.ax_phi.grid(True)
        (self.line_phi,) = self.ax_phi.plot(
            [], [], "b-", linewidth=2, label="Estimated Phi_m"
        )
        self.setup_interactive_legend(self.ax_phi, self.canvas_param)

        self.ax_l.set_title("Inductance Estimation", fontsize=10, fontweight="bold")
        self.ax_l.set_xlabel("Time (s)")
        # doi don vi uh
        self.ax_l.set_ylabel("Lq=Ld (µH)")
        self.ax_l.grid(True)
        (self.line_l,) = self.ax_l.plot([], [], "r-", linewidth=2, label="Estimated L")
        self.setup_interactive_legend(self.ax_l, self.canvas_param)

        self.fig_param.tight_layout()
        self.add_toolbar_and_click(self.canvas_param, tab4_layout, self.ax_r)

        param_info_layout = QHBoxLayout()
        # dinh dang 4 so thap phan
        self.lbl_r_info = self.create_dash_label("R: 0.0000 Ω", "#008000")
        self.lbl_phi_info = self.create_dash_label("Phi_m: 0.0000 mV.s", "#0000FF")
        self.lbl_l_info = self.create_dash_label("L: 0.00 µH", "#FF0000")
        self.lbl_temp_info = self.create_dash_label("Temp: 0.0 °C", "#333333")

        param_info_layout.addWidget(self.lbl_r_info, 1)
        param_info_layout.addWidget(self.lbl_phi_info, 1)
        param_info_layout.addWidget(self.lbl_l_info, 1)
        param_info_layout.addWidget(self.lbl_temp_info, 1)
        tab4_layout.addLayout(param_info_layout, stretch=1)

        self.tabs.addTab(tab4_widget, "Parameter Estimation")

        right_panel.addLayout(right_top_bar)
        right_panel.addWidget(self.tabs)
        main_layout.addLayout(left_panel, 3)
        main_layout.addLayout(right_panel, 7)
        # khoa ui ban dau
        self.update_ui_locked()

    # ================= CAC HAM CALIB =================
    def run_calibration(self):
        if not self.is_connected:
            self.log_to_console(
                "Please connect to FDCAN before calibration!", is_cmd=True
            )
            return

        self.calib_output_text = ""  # xoa log cu
        self.log_to_console("Preparing calibration...", is_cmd=True)
        self.btn_calib.setEnabled(False)
        self.btn_connect.setEnabled(False)

        if self.backend_process:
            try:
                self.backend_process.terminate()
                self.backend_process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.backend_process.kill()

            self.backend_process = None
            self.is_connected = False
            self.lbl_connection.setText("Status: Calibrating...")
            self.lbl_connection.setStyleSheet("color: orange; font-weight: bold;")
            self.log_to_console(
                "Backend temporarily disconnected to release COM port.", is_cmd=True
            )

        self.log_to_console("Starting calibration. Please wait...", is_cmd=True)

        self.calib_process = QProcess(self)
        self.calib_process.setProcessChannelMode(QProcess.MergedChannels)
        self.calib_process.readyReadStandardOutput.connect(self.handle_calib_output)
        self.calib_process.finished.connect(self.calib_finished)
        self.calib_process.start(
            sys.executable, ["-m", "moteus.moteus_tool", "--target", "1", "--calibrate"]
        )

    def handle_calib_output(self):
        data = (
            self.calib_process.readAllStandardOutput()
            .data()
            .decode("utf-8", errors="replace")
        )
        self.calib_output_text += data  # cong don log
        for line in data.splitlines():
            if line.strip():
                self.console_output.append(
                    f'<span style="color:#cccccc;">[Calib] {line.strip()}</span>'
                )
        self.console_output.verticalScrollBar().setValue(
            self.console_output.verticalScrollBar().maximum()
        )

    def extract_and_save_calib_data(self):
        r_val, l_val, phi_val = 0.0, 0.0, 0.0

        try:
            # tim json trong log
            match = re.search(
                r'(\{[\s\S]*"winding_resistance"[\s\S]*\})', self.calib_output_text
            )

            if match:
                json_str = match.group(1)
                parsed_data = json.loads(json_str)

                # trich xuat du lieu
                r_val = parsed_data.get("winding_resistance", 0.0)
                l_val = parsed_data.get("inductance", 0.0)
                kv_val = parsed_data.get("kv", 0.0)

                # lay so cuc
                poles_val = parsed_data.get("calibration", {}).get("poles", 14)

                # TINH TOAN PHI M
                if kv_val > 0 and poles_val > 0:
                    pole_pairs = poles_val / 2.0
                    phi_val = 60.0 / (math.sqrt(3) * 2 * math.pi * kv_val * pole_pairs)

                self.log_to_console(
                    "JSON parameters successfully extracted.", is_cmd=True
                )
            else:
                self.log_to_console(
                    "Warning: Could not find JSON block in calibration output.",
                    is_cmd=True,
                )

        except Exception as e:
            self.log_to_console(f"Error parsing calibration JSON: {e}", is_cmd=True)

        # doc calib du phong
        if os.path.exists(self.calib_file_path):
            try:
                with open(self.calib_file_path, "r") as f:
                    old_data = json.load(f)
                    if r_val == 0.0 and "r" in old_data:
                        r_val = old_data["r"]
                    if l_val == 0.0 and "l" in old_data:
                        l_val = old_data["l"]
                    if phi_val == 0.0 and "phi_m" in old_data:
                        phi_val = old_data["phi_m"]
            except Exception:
                pass

        # gia tri mac dinh an toan
        if phi_val == 0.0:
            phi_val = 0.0185

        # LUU FILE JSON
        calib_data = {"r": r_val, "l": l_val, "phi_m": phi_val}

        try:
            # dam bao ghi file
            with open(self.calib_file_path, "w") as f:
                json.dump(calib_data, f, indent=4)
            # LOG KEM DON VI (DOI PHI SANG mV.s)
            self.log_to_console(
                f"Updated Params: R={calib_data['r']:.4f} Ω, L={calib_data['l'] * 1e6:.4f} µH, Phi_m={calib_data['phi_m'] * 1000.0:.4f} mV.s",
                is_cmd=True,
            )
        except Exception as e:
            self.log_to_console(
                f"Error writing to {self.calib_file_path}: {e}", is_cmd=True
            )

    def calib_finished(self, exit_code, exit_status):
        if exit_code == 0:
            self.log_to_console("Calibration completed successfully!", is_cmd=True)
            self.extract_and_save_calib_data()  # GOI HAM LUU THONG SO
        else:
            self.log_to_console(
                f"Calibration finished with error code: {exit_code}", is_cmd=True
            )

        self.log_to_console("Restarting Backend...", is_cmd=True)
        self.calib_process.deleteLater()
        self.calib_process = None
        self.btn_connect.setEnabled(True)
        self.start_backend()
        QTimer.singleShot(1500, self.connect_fdcan)

    def copy_to_clipboard(self):
        pixmap = self.tabs.grab()
        clipboard = QApplication.clipboard()
        clipboard.setPixmap(pixmap)
        tab_name = self.tabs.tabText(self.tabs.currentIndex())
        self.log_to_console(f"Image of '{tab_name}' copied to Clipboard.", is_cmd=True)

    def change_mode(self):
        selected_mode = self.mode_selector.currentText()
        if selected_mode != self.current_mode:
            self.current_mode = selected_mode
            mode_cmd = "sensorless" if "Sensorless" in selected_mode else "normal"
            command = {"type": "set_mode", "mode": mode_cmd}
            self.pub_socket.send_string(json.dumps(command))
            self.log_to_console(f"{selected_mode}", is_cmd=True)

    def connect_fdcan(self):
        # bat tat ket noi
        if not self.is_connected and not self.waiting_for_connection:
            # che do ket noi
            self.log_to_console("Reinitializing connection...", is_cmd=True)
            self.clear_all_dashed_lines()
            self.last_motor_error_code = 0
            self.pid_dq_request_sent = False
            if self.backend_process:
                try:
                    self.backend_process.terminate()
                    self.backend_process.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    self.backend_process.kill()
            self.backend_process = None
            self.lbl_connection.setText("Status: Restarting...")
            self.lbl_connection.setStyleSheet("color: orange; font-weight: bold;")
            self.start_backend()
            QTimer.singleShot(1500, self._send_connect_cmd)
            self.update_ui_locked()
        elif self.is_connected:
            # che do ngat ket noi
            self.log_to_console("Disconnecting FDCAN...", is_cmd=True)
            self.clear_all_dashed_lines()
            try:
                self.pub_socket.send_string(json.dumps({"type": "disconnect_fdcan"}))
            except Exception:
                pass

            # GIAI PHONG TAI NGUYEN COM FDCAN
            if self.backend_process:
                try:
                    self.backend_process.terminate()
                    self.backend_process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    self.backend_process.kill()
                self.backend_process = None

            self.is_connected = False
            self.waiting_for_connection = False
            self.lbl_connection.setText("Status: Disconnected")
            self.lbl_connection.setStyleSheet("color: red; font-weight: bold;")
            self.btn_calib.setEnabled(False)
            self.btn_run_script.setEnabled(False)
            self.update_ui_locked()

    def _send_connect_cmd(self):
        # giu nut ket noi hieu luc
        command = {"type": "connect_fdcan"}
        self.pub_socket.send_string(json.dumps(command))
        self.connect_request_time = time.time()
        self.waiting_for_connection = True
        self.lbl_connection.setText("Status: Connecting...")
        self.lbl_connection.setStyleSheet("color: orange; font-weight: bold;")
        self.log_to_console("Sending FDCAN connection request...", is_cmd=True)

    def log_to_console(self, text, is_cmd=False):
        prefix = ">> " if is_cmd else ""
        color = "#00ffff" if is_cmd else "#00ff00"
        self.console_output.append(
            f'<span style="color:{color};">{prefix}{text}</span>'
        )
        self.console_output.verticalScrollBar().setValue(
            self.console_output.verticalScrollBar().maximum()
        )

    def refresh_scripts_list(self):
        self.script_combo.clear()
        search_pattern = os.path.join(self.scripts_folder, "*.py")
        files = glob.glob(search_pattern)
        for f in files:
            self.script_combo.addItem(os.path.basename(f))
        if not files:
            self.script_combo.addItem("-- No files found --")

    def run_selected_script(self):
        script_name = self.script_combo.currentText()
        if script_name and script_name != "-- No files found --":
            self.stop_script()
            script_path = os.path.join(self.scripts_folder, script_name)
            try:
                self.current_script_process = subprocess.Popen(
                    [sys.executable, script_path]
                )
                self.lbl_script_status.setText(f"Status: Running '{script_name}'")
                self.lbl_script_status.setStyleSheet("color: green; font-weight: bold;")
                self.log_to_console(f"Script loaded and started: {script_name}")
            except Exception as e:
                self.log_to_console(f"Error starting script: {e}")

    def stop_script(self):
        if self.current_script_process and self.current_script_process.poll() is None:
            self.current_script_process.terminate()
            stop_cmd = {"type": "d_stop"}
            self.pub_socket.send_string(json.dumps(stop_cmd))
            self.lbl_script_status.setText("Status: Script stopped & Motor STOPPED")
            self.lbl_script_status.setStyleSheet("color: red; font-weight: bold;")

    def send_d_pos(self):
        try:
            pos = float(self.pos_input.text())
            vel = float(self.vel_input.text())
            torque = float(self.torque_input.text())
            opts = self.opts_input.text().strip()

            cmd = f"d pos {pos} {vel} {torque}"
            if opts:
                cmd += f" {opts}"

            command = {"type": "raw", "cmd": cmd}
            self.pub_socket.send_string(json.dumps(command))
            self.log_to_console(cmd, is_cmd=True)
        except ValueError:
            self.log_to_console("Error: Invalid input values!")

    def request_pid_dq_once(self):
        # THEM CHECK is_connected LAN CHOT DE AN TOAN TUYET DOI
        if self.is_connected and not self.pid_dq_request_sent:
            try:
                self.pub_socket.send_string(
                    json.dumps({"type": "raw", "cmd": "conf get servo.pid_dq.kp"})
                )
                self.pub_socket.send_string(
                    json.dumps({"type": "raw", "cmd": "conf get servo.pid_dq.ki"})
                )
                self.pid_dq_request_sent = True
                self.log_to_console("Requesting current PID DQ values...", is_cmd=True)
            except Exception as e:
                self.log_to_console(f"Error requesting PID DQ: {e}")

    def send_pid_dq(self):
        try:
            kp_text = self.kp_d_input.text().strip()
            ki_text = self.ki_d_input.text().strip()

            if kp_text:
                kp_val = float(kp_text)
                self.pub_socket.send_string(
                    json.dumps(
                        {"type": "raw", "cmd": f"conf set servo.pid_dq.kp {kp_val}"}
                    )
                )
                self.log_to_console(f"conf set servo.pid_dq.kp {kp_val}", is_cmd=True)

            if ki_text:
                ki_val = float(ki_text)
                self.pub_socket.send_string(
                    json.dumps(
                        {"type": "raw", "cmd": f"conf set servo.pid_dq.ki {ki_val}"}
                    )
                )
                self.log_to_console(f"conf set servo.pid_dq.ki {ki_val}", is_cmd=True)

            if not kp_text and not ki_text:
                self.log_to_console("No PID DQ values entered.")
        except ValueError:
            self.log_to_console("Error: Invalid PID DQ values!")

    def send_d_stop(self):
        self.stop_script()
        command = {"type": "d_stop"}
        self.pub_socket.send_string(json.dumps(command))
        self.log_to_console("Emergency stop sent to motor.", is_cmd=True)

    def send_console_command(self):
        cmd = self.console_input.text().strip()
        if cmd:
            if not self.is_connected:
                self.log_to_console("FDCAN is not connected. Command was not sent.")
                return
            if not self.validate_command(cmd):
                self.log_to_console(f"Invalid syntax: {cmd}")
                return
            command = {"type": "raw", "cmd": cmd}
            self.pub_socket.send_string(json.dumps(command))
            self.log_to_console(cmd, is_cmd=True)
            self.console_input.clear()

    def validate_command(self, cmd: str) -> bool:
        parts = cmd.split()
        if len(parts) == 0:
            return False
        if parts[0] == "d" and len(parts) >= 2:
            if parts[1] == "pos":
                if len(parts) < 5:
                    return False
                try:
                    float(parts[2])
                    float(parts[3])
                    float(parts[4])
                except:
                    return False
                valid_prefix = ["p", "d", "i", "s", "f", "t", "v", "a", "o", "c", "b"]
                for opt in parts[5:]:
                    if len(opt) < 2:
                        return False
                    if opt[0] not in valid_prefix:
                        return False
                    try:
                        float(opt[1:])
                    except:
                        return False
                return True
        if cmd == "d stop":
            return True
        return True

    def clear_console(self):
        self.console_output.clear()

    def auto_zoom_axis(self, ax, *data_lists):
        recent_vals = []
        for d in data_lists:
            if len(d) > 0:
                recent_vals.extend(list(d)[-10000:])

        clean_vals = [v for v in recent_vals if np.isfinite(v)]

        if clean_vals:
            min_y = min(clean_vals)
            max_y = max(clean_vals)

            span = max_y - min_y
            if span > 1e-8:
                ax.set_ylim(min_y - span * 0.1, max_y + span * 0.1)
            else:
                if abs(max_y) > 1e-8:
                    ax.set_ylim(max_y * 0.9, max_y * 1.1)
                else:
                    ax.set_ylim(-0.1, 0.1)

    def update_ui_locked(self):
        enabled = bool(self.is_connected)
        try:
            self.pos_input.setEnabled(enabled)
            self.vel_input.setEnabled(enabled)
            self.torque_input.setEnabled(enabled)
            self.opts_input.setEnabled(enabled)
        except Exception:
            pass
        try:
            self.btn_send_pos.setEnabled(enabled)
            self.btn_stop.setEnabled(enabled)
        except Exception:
            pass
        try:
            self.console_input.setEnabled(enabled)
        except Exception:
            pass
        try:
            self.script_combo.setEnabled(enabled)
            self.btn_run_script.setEnabled(enabled and self.script_combo.count() > 0)
            # giu nut refresh hieu luc
            if hasattr(self, "btn_refresh"):
                self.btn_refresh.setEnabled(True)
        except Exception:
            pass

    def fault_code_to_text(self, code):
        """CHUYEN DOI MA LOI TVIEW"""
        m = {
            0: "no fault",
            1: "overcurrent",
            2: "overvoltage",
            3: "undervoltage",
            33: "gate driver fault",
            39: "outside limit",
        }
        return m.get(code, f"unknown fault code {code}")

    def update_data_and_plot(self):
        new_data = False
        last_data = None
        base_auto_scroll = self.chk_autoscroll.isChecked()

        while True:
            try:
                message = self.sub_socket.recv_string(flags=zmq.NOBLOCK)
                data = json.loads(message)

                msg_type = data.get("type")
                if msg_type == "heartbeat":
                    self.last_heartbeat = time.time()

                    # CHI XAC NHAN KET NOI KHI THUC SU DANG CHO (WAITING) DE TRANH HOI PID KHI DISCONNECT
                    if self.waiting_for_connection:
                        self.waiting_for_connection = False
                        if not self.is_connected:
                            self.is_connected = True
                            self.lbl_connection.setText("Status: Connected")
                            self.lbl_connection.setStyleSheet(
                                "color: green; font-weight: bold;"
                            )
                            self.btn_calib.setEnabled(True)
                            self.btn_run_script.setEnabled(True)
                            self.update_ui_locked()

                            self.connection_start_time = None
                            self.clear_plot_data()
                            self.last_motor_error_code = 0
                            self.pid_dq_request_sent = False

                            QTimer.singleShot(500, self.request_pid_dq_once)
                    continue

                if msg_type == "log":
                    msg_text = f"{data.get('msg')}"
                    self.log_to_console(msg_text)
                    continue

                if msg_type == "pid_dq":
                    kp_val = data.get("kp")
                    ki_val = data.get("ki")
                    if kp_val is not None and not self.kp_d_input.hasFocus():
                        self.kp_d_input.setText(str(kp_val))
                    if ki_val is not None and not self.ki_d_input.hasFocus():
                        self.ki_d_input.setText(str(ki_val))
                    continue

                if "time" in data:
                    if self.connection_start_time is None:
                        self.connection_start_time = data["time"]
                    adjusted_time = data["time"] - self.connection_start_time
                    self.times.append(adjusted_time)

                    self.cmd_pos.append(data.get("command_position", 0))
                    self.act_pos.append(data.get("actual_position", 0))
                    self.sl_pos.append(data.get("sensorless_position", 0))

                    self.cmd_vel.append(data.get("command_velocity", 0))
                    self.act_vel.append(data.get("actual_velocity", 0))
                    self.sl_vel.append(data.get("sensorless_velocity", 0))

                    # tinh toan loi tuong thich
                    self.err_pos.append(self.cmd_pos[-1] - self.act_pos[-1])
                    self.err_vel.append(self.cmd_vel[-1] - self.act_vel[-1])

                    # TINH TOAN LOI DOC LAP
                    try:
                        as_pos = self.act_pos[-1] - self.sl_pos[-1]
                        sc_pos = self.sl_pos[-1] - self.cmd_pos[-1]
                        self.err_as_pos.append(as_pos)
                        self.err_sc_pos.append(sc_pos)
                        if len(self.err_as_pos) > 1:
                            self.var_as_pos.append(
                                float(np.var(list(self.err_as_pos)[-1000:]))
                            )
                        else:
                            self.var_as_pos.append(0.0)
                        if len(self.err_sc_pos) > 1:
                            self.var_sc_pos.append(
                                float(np.var(list(self.err_sc_pos)[-1000:]))
                            )
                        else:
                            self.var_sc_pos.append(0.0)
                    except Exception:
                        pass

                    try:
                        as_vel = self.act_vel[-1] - self.sl_vel[-1]
                        sc_vel = self.sl_vel[-1] - self.cmd_vel[-1]
                        self.err_as_vel.append(as_vel)
                        self.err_sc_vel.append(sc_vel)
                        if len(self.err_as_vel) > 1:
                            self.var_as_vel.append(
                                float(np.var(list(self.err_as_vel)[-1000:]))
                            )
                        else:
                            self.var_as_vel.append(0.0)
                        if len(self.err_sc_vel) > 1:
                            self.var_sc_vel.append(
                                float(np.var(list(self.err_sc_vel)[-1000:]))
                            )
                        else:
                            self.var_sc_vel.append(0.0)
                    except Exception:
                        pass

                    v_pos = np.var(self.err_pos) if len(self.err_pos) > 1 else 0
                    v_vel = np.var(self.err_vel) if len(self.err_vel) > 1 else 0
                    self.var_pos_list.append(v_pos)
                    self.var_vel_list.append(v_vel)

                    self.cmd_torque.append(data.get("command_torque", 0))
                    self.act_torque.append(data.get("actual_torque", 0))

                    self.iq_values.append(data.get("i_q", 0))
                    self.id_values.append(data.get("i_d", 0))

                    self.r_values.append(data.get("r_hat", 0))
                    # DOI SANG mV.s O KHAN UPDATE DO THI GUI
                    self.phi_values.append(data.get("phi_m", 0) * 1000.0)

                    # chuyen don vi sang uh
                    l_uh = data.get("L_hat", 0) * 1e6
                    self.l_values.append(l_uh)
                    self.latest_temp = data.get("temp", 0)

                    # KIEM TRA LOI THEO DOI PHAN CUNG
                    try:
                        mp = data.get("motor_position") or data.get(
                            "telemetry", {}
                        ).get("motor_position")
                        if mp and isinstance(mp, dict):
                            err_code = int(mp.get("error", 0) or 0)
                            if err_code != 0:
                                if err_code != self.last_motor_error_code:
                                    # uu tien loi tu backend
                                    raw_err_text = (
                                        mp.get("error_text")
                                        or mp.get("description")
                                        or ""
                                    ).strip()
                                    if (
                                        raw_err_text
                                        and not raw_err_text.lower().startswith(
                                            "unknown"
                                        )
                                    ):
                                        err_text = raw_err_text.lower()
                                    else:
                                        err_text = self.fault_code_to_text(err_code)
                                    display = f"fault {err_code} &lt;{err_text}&gt;"  # xu ly ky tu hien thi
                                    self.log_to_console(display, is_cmd=True)
                                self.last_motor_error_code = err_code
                            else:
                                self.last_motor_error_code = 0
                    except Exception:
                        pass

                    last_data = data
                    new_data = True

            except zmq.Again:
                break
            except Exception as e:
                print(f"Error processing message: {e}")
                break

        now = time.time()
        if self.waiting_for_connection and self.connect_request_time:
            if now - self.connect_request_time > 2.0:
                self.waiting_for_connection = False
                self.lbl_connection.setText("Status: Device not found")
                self.lbl_connection.setStyleSheet("color: red; font-weight: bold;")
                self.log_to_console("Device not found!")
                self.update_ui_locked()

        if self.is_connected and (now - self.last_heartbeat > 1.0):
            self.is_connected = False
            self.lbl_connection.setText("Status: Connection lost")
            self.lbl_connection.setStyleSheet("color: red; font-weight: bold;")
            self.btn_calib.setEnabled(False)
            self.btn_run_script.setEnabled(False)
            self.update_ui_locked()
            self.log_to_console("FDCAN connection lost")

        if new_data and last_data:
            self.lbl_cmd_pos.setText(f"Cmd: {self.cmd_pos[-1]:.3f} rad")
            self.lbl_act_pos.setText(f"Act: {self.act_pos[-1]:.3f} rad")
            self.lbl_sl_pos.setText(f"SL: {self.sl_pos[-1]:.3f} rad")

            self.lbl_cmd_vel.setText(f"Cmd: {self.cmd_vel[-1]:.3f} rad/s")
            self.lbl_act_vel.setText(f"Act: {self.act_vel[-1]:.3f} rad/s")
            self.lbl_sl_vel.setText(f"SL: {self.sl_vel[-1]:.3f} rad/s")

            self.lbl_act_torque_val.setText(f"Actual: {self.act_torque[-1]:.3f} Nm")
            self.lbl_cmd_torque_val.setText(f"Command: {self.cmd_torque[-1]:.3f} Nm")

            self.lbl_iq_val.setText(f"i_q: {self.iq_values[-1]:.3f} A")
            self.lbl_id_val.setText(f"i_d: {self.id_values[-1]:.3f} A")
            # 4 so thap phan r va phi m (da doi don vi tren lbl)
            self.lbl_r_info.setText(f"R: {self.r_values[-1]:.4f} Ω")
            self.lbl_phi_info.setText(f"Phi_m: {self.phi_values[-1]:.4f} mV.s")
            # cap nhat don vi uh
            self.lbl_l_info.setText(f"L: {self.l_values[-1]:.4f} µH")
            self.lbl_temp_info.setText(f"Temp: {self.latest_temp:.1f} °C")

            current_tab = self.tabs.currentIndex()
            t_axis = list(self.times)

            valid_x = len(t_axis) > 1 and t_axis[-1] > t_axis[0]

            if current_tab == 0:
                self.line_cmd_pos.set_data(t_axis, list(self.cmd_pos))
                self.line_act_pos.set_data(t_axis, list(self.act_pos))
                self.line_sl_pos.set_data(t_axis, list(self.sl_pos))

                auto_scroll_pos = (
                    base_auto_scroll and self.canvas_pos.toolbar.mode == ""
                )
                if auto_scroll_pos and valid_x:
                    self.ax_pos.set_xlim(t_axis[0], t_axis[-1])
                    self.ax_pos.relim()
                    self.ax_pos.autoscale_view(scalex=False, scaley=True)
                self.canvas_pos.draw_idle()

                self.line_cmd_vel.set_data(t_axis, list(self.cmd_vel))
                self.line_act_vel.set_data(t_axis, list(self.act_vel))
                self.line_sl_vel.set_data(t_axis, list(self.sl_vel))

                auto_scroll_vel = (
                    base_auto_scroll and self.canvas_vel.toolbar.mode == ""
                )
                if auto_scroll_vel and valid_x:
                    self.ax_vel.set_xlim(t_axis[0], t_axis[-1])
                    self.ax_vel.relim()
                    self.ax_vel.autoscale_view(scalex=False, scaley=True)
                self.canvas_vel.draw_idle()

            elif current_tab == 1:
                # gia tri loi va phuong sai moi nhat
                as_pos = self.err_as_pos[-1] if self.err_as_pos else 0.0
                sc_pos = self.err_sc_pos[-1] if self.err_sc_pos else 0.0
                var_as_p = self.var_as_pos[-1] if self.var_as_pos else 0.0
                var_sc_p = self.var_sc_pos[-1] if self.var_sc_pos else 0.0

                as_vel = self.err_as_vel[-1] if self.err_as_vel else 0.0
                sc_vel = self.err_sc_vel[-1] if self.err_sc_vel else 0.0
                var_as_v = self.var_as_vel[-1] if self.var_as_vel else 0.0
                var_sc_v = self.var_sc_vel[-1] if self.var_sc_vel else 0.0

                self.lbl_err_pos_as.setText(f"Act-SL: {as_pos:.4f} rad")
                self.lbl_err_pos_sc.setText(f"SL-Cmd: {sc_pos:.4f} rad")
                self.lbl_var_pos_as.setText(f"Var(Act-SL): {var_as_p:.2e}")
                self.lbl_var_pos_sc.setText(f"Var(SL-Cmd): {var_sc_p:.2e}")

                self.lbl_err_vel_as.setText(f"Act-SL: {as_vel:.4f} rad/s")
                self.lbl_err_vel_sc.setText(f"SL-Cmd: {sc_vel:.4f} rad/s")
                self.lbl_var_vel_as.setText(f"Var(Act-SL): {var_as_v:.2e}")
                self.lbl_var_vel_sc.setText(f"Var(SL-Cmd): {var_sc_v:.2e}")

                self.line_err_pos_as.set_data(t_axis, list(self.err_as_pos))
                self.line_err_pos_sc.set_data(t_axis, list(self.err_sc_pos))
                self.line_var_pos_as.set_data(t_axis, list(self.var_as_pos))
                self.line_var_pos_sc.set_data(t_axis, list(self.var_sc_pos))

                auto_scroll_err_pos = (
                    base_auto_scroll and self.canvas_err_pos.toolbar.mode == ""
                )
                if auto_scroll_err_pos and valid_x:
                    self.ax_err_pos.set_xlim(t_axis[0], t_axis[-1])
                    self.auto_zoom_axis(
                        self.ax_err_pos, self.err_as_pos, self.err_sc_pos
                    )
                    self.auto_zoom_axis(
                        self.ax_var_pos, self.var_as_pos, self.var_sc_pos
                    )
                self.canvas_err_pos.draw_idle()

                self.line_err_vel_as.set_data(t_axis, list(self.err_as_vel))
                self.line_err_vel_sc.set_data(t_axis, list(self.err_sc_vel))
                self.line_var_vel_as.set_data(t_axis, list(self.var_as_vel))
                self.line_var_vel_sc.set_data(t_axis, list(self.var_sc_vel))

                auto_scroll_err_vel = (
                    base_auto_scroll and self.canvas_err_vel.toolbar.mode == ""
                )
                if auto_scroll_err_vel and valid_x:
                    self.ax_err_vel.set_xlim(t_axis[0], t_axis[-1])
                    self.auto_zoom_axis(
                        self.ax_err_vel, self.err_as_vel, self.err_sc_vel
                    )
                    self.auto_zoom_axis(
                        self.ax_var_vel, self.var_as_vel, self.var_sc_vel
                    )
                self.canvas_err_vel.draw_idle()

            elif current_tab == 2:
                self.line_act_torque.set_data(t_axis, list(self.act_torque))
                self.line_cmd_torque.set_data(t_axis, list(self.cmd_torque))

                auto_scroll_torque = (
                    base_auto_scroll and self.canvas_torque.toolbar.mode == ""
                )
                if auto_scroll_torque and valid_x:
                    self.ax_torque.set_xlim(t_axis[0], t_axis[-1])
                    self.auto_zoom_axis(
                        self.ax_torque, self.act_torque, self.cmd_torque
                    )
                self.canvas_torque.draw_idle()

                self.line_iq.set_data(t_axis, list(self.iq_values))
                self.line_id.set_data(t_axis, list(self.id_values))

                auto_scroll_curr = (
                    base_auto_scroll and self.canvas_curr.toolbar.mode == ""
                )
                if auto_scroll_curr and valid_x:
                    self.ax_curr.set_xlim(t_axis[0], t_axis[-1])
                    self.auto_zoom_axis(self.ax_curr, self.iq_values, self.id_values)
                self.canvas_curr.draw_idle()

            elif current_tab == 3:
                self.line_r.set_data(t_axis, list(self.r_values))
                self.line_phi.set_data(t_axis, list(self.phi_values))
                self.line_l.set_data(t_axis, list(self.l_values))

                auto_scroll_param = (
                    base_auto_scroll and self.canvas_param.toolbar.mode == ""
                )
                if auto_scroll_param and valid_x:
                    self.ax_r.set_xlim(t_axis[0], t_axis[-1])
                    self.auto_zoom_axis(self.ax_r, self.r_values)

                    self.ax_phi.set_xlim(t_axis[0], t_axis[-1])
                    self.auto_zoom_axis(self.ax_phi, self.phi_values)

                    self.ax_l.set_xlim(t_axis[0], t_axis[-1])
                    self.auto_zoom_axis(self.ax_l, self.l_values)

                self.canvas_param.draw_idle()


if __name__ == "__main__":
    if hasattr(Qt, "AA_EnableHighDpiScaling"):
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, "AA_UseHighDpiPixmaps"):
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    window = ControllerGUI()
    window.show()
    sys.exit(app.exec_())
