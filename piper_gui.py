#!/usr/bin/env python3
"""Piper X ROS 2 operator GUI.

This application is intentionally a ROS-only front end.  The existing
``agx_arm_ctrl`` node remains the only process that talks to SocketCAN.
"""

from __future__ import annotations

import math
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from PyQt5.QtCore import QObject, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPalette
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QButtonGroup,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHeaderView,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QStackedWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

import rclpy
from agx_arm_msgs.msg import AgxArmStatus, GripperStatus
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool, Trigger

from arm_simulation import PiperSimulationWidget


APP_TITLE = "Piper ROS 2 GUI"
JOINT_NAMES: Tuple[str, ...] = tuple(f"joint{i}" for i in range(1, 7))

# Piper X limits from the official piper_x_description.urdf.
JOINT_LIMITS_RAD: Tuple[Tuple[float, float], ...] = (
    (-2.6179938, 2.6179938),
    (0.0, 3.1415926),
    (-2.9670597, 0.0),
    (-1.553343, 1.553343),
    (-1.553343, 1.553343),
    (-2.0943951, 2.0943951),
)

FEEDBACK_STALE_SECONDS = 1.0
MAX_JOINT_STEP_DEG = 20.0
MAX_JOINT_STEP_RAD = math.radians(MAX_JOINT_STEP_DEG)
GRIPPER_WIDTH_MIN_M = 0.0
GRIPPER_WIDTH_MAX_M = 0.1
GRIPPER_FORCE_MIN_N = 0.5
GRIPPER_FORCE_MAX_N = 3.0

CTRL_MODE_TEXT = {
    0: "待机",
    1: "CAN 控制",
    2: "示教模式",
    3: "以太网控制",
    4: "Wi-Fi 控制",
    5: "遥控模式",
    6: "联动示教输入",
    7: "离线轨迹模式",
    8: "TCP 控制",
}

ARM_STATUS_TEXT = {
    0: "正常",
    1: "急停",
    2: "无解",
    3: "奇异点",
    4: "目标角度超限",
    5: "关节通信异常",
    6: "关节刹车未释放",
    7: "发生碰撞",
    8: "示教拖动超速",
    9: "关节状态异常",
    10: "其他异常",
    11: "示教记录中",
    12: "示教执行中",
    13: "示教暂停",
    14: "主控 NTC 过温",
    15: "释放电阻 NTC 过温",
}

TEACH_STATUS_TEXT = {
    0: "关闭",
    1: "示教记录中",
    2: "已退出示教",
    3: "执行示教轨迹",
    4: "暂停执行",
    5: "继续执行",
}

MOVE_MODE_TEXT = {
    0: "MOVE P",
    1: "MOVE J",
    2: "MOVE L",
    3: "MOVE C",
    4: "MOVE MIT",
    5: "MOVE CPV",
    255: "未选择",
}


@dataclass(frozen=True)
class CanState:
    exists: bool
    up: bool
    bitrate: Optional[int]
    error_state: str
    detail: str

    @property
    def ready(self) -> bool:
        return self.exists and self.up and self.bitrate == 1_000_000


def read_can_state(interface: str = "can0") -> CanState:
    """Read SocketCAN state without changing the interface."""
    try:
        result = subprocess.run(
            ["ip", "-details", "link", "show", interface],
            check=False,
            capture_output=True,
            text=True,
            timeout=1.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return CanState(False, False, None, "", str(exc))

    output = (result.stdout or result.stderr).strip()
    if result.returncode != 0:
        return CanState(False, False, None, "", output)

    first_line = output.splitlines()[0] if output else ""
    flags = ""
    if "<" in first_line and ">" in first_line:
        flags = first_line.split("<", 1)[1].split(">", 1)[0]
    up = "UP" in {flag.strip() for flag in flags.split(",")}

    bitrate: Optional[int] = None
    error_state = ""
    tokens = output.replace("\n", " ").split()
    for index, token in enumerate(tokens):
        if token == "bitrate" and index + 1 < len(tokens):
            try:
                bitrate = int(tokens[index + 1])
            except ValueError:
                pass
        if token == "state" and index + 1 < len(tokens):
            candidate = tokens[index + 1]
            if candidate.startswith("ERROR-"):
                error_state = candidate

    return CanState(True, up, bitrate, error_state, output)


def ordered_joint_values(msg: JointState) -> Optional[List[float]]:
    """Return joint1..joint6 values, or None when feedback is incomplete."""
    positions = {
        name: float(msg.position[index])
        for index, name in enumerate(msg.name)
        if index < len(msg.position) and name in JOINT_NAMES
    }
    if any(name not in positions for name in JOINT_NAMES):
        return None
    return [positions[name] for name in JOINT_NAMES]


def validate_joint_target(
    current: Sequence[float], target: Sequence[float]
) -> Tuple[bool, str, float]:
    """Validate bounds and maximum one-command step."""
    if len(current) != 6 or len(target) != 6:
        return False, "关节数据必须包含 joint1～joint6", 0.0

    for index, value in enumerate(target):
        lower, upper = JOINT_LIMITS_RAD[index]
        if not math.isfinite(value):
            return False, f"joint{index + 1} 目标值不是有效数字", 0.0
        if value < lower or value > upper:
            return (
                False,
                f"joint{index + 1} 超出限制 "
                f"[{math.degrees(lower):.1f}°, {math.degrees(upper):.1f}°]",
                0.0,
            )

    max_delta = max(abs(goal - actual) for actual, goal in zip(current, target))
    if max_delta > MAX_JOINT_STEP_RAD + 1e-9:
        return (
            False,
            f"单次最大变化 {math.degrees(max_delta):.2f}°，"
            f"超过安全限制 {MAX_JOINT_STEP_DEG:.0f}°。"
            "请同步当前位置后减小目标。",
            max_delta,
        )
    return True, "", max_delta


def quaternion_to_euler_degrees(
    x: float, y: float, z: float, w: float
) -> Tuple[float, float, float]:
    """Convert a ROS quaternion to roll, pitch and yaw in degrees."""
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) if abs(sin_pitch) >= 1 else math.asin(sin_pitch)

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def validate_gripper_target(width_m: float, force_n: float) -> Tuple[bool, str]:
    """Validate the public AgxGripper width/force limits."""
    if not math.isfinite(width_m) or not (
        GRIPPER_WIDTH_MIN_M <= width_m <= GRIPPER_WIDTH_MAX_M
    ):
        return False, "夹爪开度必须在 0–100 mm 之间"
    if not math.isfinite(force_n) or not (
        GRIPPER_FORCE_MIN_N <= force_n <= GRIPPER_FORCE_MAX_N
    ):
        return False, "夹持力度必须在 0.5–3.0 N 之间"
    return True, ""


class RosBridge(QObject):
    joint_received = pyqtSignal(object)
    pose_received = pyqtSignal(object)
    status_received = pyqtSignal(object)
    gripper_received = pyqtSignal(object)
    service_finished = pyqtSignal(str, bool, str)
    log_message = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = Node("piper_operator_gui")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # The driver publishes at up to 200 Hz.  Keeping only the newest frame
        # prevents thousands of queued Qt signals from making the UI appear
        # hung while a page is switched or the 3D view is being repainted.
        self._feedback_lock = threading.Lock()
        self._latest_joint_msg = None
        self._latest_pose_msg = None
        self._latest_status_msg = None
        self._latest_gripper_msg = None

        self.node.create_subscription(
            JointState,
            "/feedback/joint_states",
            lambda msg: self._store_feedback("joint", msg),
            10,
        )
        self.node.create_subscription(
            PoseStamped,
            "/feedback/tcp_pose",
            lambda msg: self._store_feedback("pose", msg),
            10,
        )
        self.node.create_subscription(
            AgxArmStatus,
            "/feedback/arm_status",
            lambda msg: self._store_feedback("status", msg),
            10,
        )
        self.node.create_subscription(
            GripperStatus,
            "/feedback/gripper_status",
            lambda msg: self._store_feedback("gripper", msg),
            10,
        )

        self.feedback_timer = QTimer(self)
        self.feedback_timer.setInterval(50)  # GUI feedback is capped at 20 Hz.
        self.feedback_timer.timeout.connect(self._flush_feedback)
        self.feedback_timer.start()

        self.move_j_publisher = self.node.create_publisher(
            JointState, "/control/move_j", 10
        )
        self.control_joint_publisher = self.node.create_publisher(
            JointState, "/control/joint_states", 10
        )
        self.set_bool_clients: Dict[str, object] = {}
        self.empty_clients: Dict[str, object] = {}
        self.trigger_clients: Dict[str, object] = {}

        self._thread = threading.Thread(
            target=self.executor.spin,
            name="piper-gui-ros-spin",
            daemon=True,
        )
        self._thread.start()

    def _store_feedback(self, kind: str, msg: object) -> None:
        """Store only the newest ROS frame; called by the ROS executor thread."""
        with self._feedback_lock:
            if kind == "joint":
                self._latest_joint_msg = msg
            elif kind == "pose":
                self._latest_pose_msg = msg
            elif kind == "status":
                self._latest_status_msg = msg
            else:
                self._latest_gripper_msg = msg

    def _flush_feedback(self) -> None:
        """Deliver one coalesced feedback set on the Qt GUI thread."""
        with self._feedback_lock:
            joint_msg = self._latest_joint_msg
            pose_msg = self._latest_pose_msg
            status_msg = self._latest_status_msg
            gripper_msg = self._latest_gripper_msg
            self._latest_joint_msg = None
            self._latest_pose_msg = None
            self._latest_status_msg = None
            self._latest_gripper_msg = None
        if joint_msg is not None:
            self.joint_received.emit(joint_msg)
        if pose_msg is not None:
            self.pose_received.emit(pose_msg)
        if status_msg is not None:
            self.status_received.emit(status_msg)
        if gripper_msg is not None:
            self.gripper_received.emit(gripper_msg)

    def _set_bool_client(self, service_name: str):
        if service_name not in self.set_bool_clients:
            self.set_bool_clients[service_name] = self.node.create_client(
                SetBool, service_name
            )
        return self.set_bool_clients[service_name]

    def _empty_client(self, service_name: str):
        if service_name not in self.empty_clients:
            self.empty_clients[service_name] = self.node.create_client(
                Empty, service_name
            )
        return self.empty_clients[service_name]

    def _trigger_client(self, service_name: str):
        if service_name not in self.trigger_clients:
            self.trigger_clients[service_name] = self.node.create_client(
                Trigger, service_name
            )
        return self.trigger_clients[service_name]

    def call_set_bool(
        self, tag: str, service_name: str, value: bool
    ) -> None:
        client = self._set_bool_client(service_name)
        if not client.service_is_ready() and not client.wait_for_service(
            timeout_sec=0.75
        ):
            self.service_finished.emit(
                tag, False, f"服务 {service_name} 不可用；请先启动 ROS 驱动"
            )
            return

        request = SetBool.Request()
        request.data = value
        future = client.call_async(request)

        def done_callback(done_future) -> None:
            try:
                response = done_future.result()
                self.service_finished.emit(
                    tag, bool(response.success), str(response.message)
                )
            except Exception as exc:  # ROS future exceptions are runtime data.
                self.service_finished.emit(tag, False, str(exc))

        future.add_done_callback(done_callback)

    def call_empty(self, tag: str, service_name: str) -> None:
        client = self._empty_client(service_name)
        if not client.service_is_ready() and not client.wait_for_service(
            timeout_sec=0.75
        ):
            self.service_finished.emit(
                tag, False, f"服务 {service_name} 不可用；请先启动 ROS 驱动"
            )
            return

        future = client.call_async(Empty.Request())

        def done_callback(done_future) -> None:
            try:
                done_future.result()
                self.service_finished.emit(tag, True, f"{service_name} 调用完成")
            except Exception as exc:
                self.service_finished.emit(tag, False, str(exc))

        future.add_done_callback(done_callback)

    def call_trigger(self, tag: str, service_name: str) -> None:
        client = self._trigger_client(service_name)
        if not client.service_is_ready() and not client.wait_for_service(
            timeout_sec=0.75
        ):
            self.service_finished.emit(
                tag, False, f"服务 {service_name} 不可用；请重新编译并启动最新 ROS 驱动"
            )
            return

        future = client.call_async(Trigger.Request())

        def done_callback(done_future) -> None:
            try:
                response = done_future.result()
                self.service_finished.emit(
                    tag, bool(response.success), str(response.message)
                )
            except Exception as exc:
                self.service_finished.emit(tag, False, str(exc))

        future.add_done_callback(done_callback)

    def publish_joint_target(self, positions: Sequence[float]) -> bool:
        if self.move_j_publisher.get_subscription_count() < 1:
            self.log_message.emit("发送失败：/control/move_j 没有驱动订阅者")
            return False

        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(value) for value in positions]
        self.move_j_publisher.publish(msg)
        self.log_message.emit("已发送一次 MOVE J 关节目标")
        return True

    def publish_gripper_target(self, width_m: float, force_n: float) -> bool:
        if self.control_joint_publisher.get_subscription_count() < 1:
            self.log_message.emit("发送失败：/control/joint_states 没有驱动订阅者")
            return False
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = ["gripper"]
        msg.position = [float(width_m)]
        msg.effort = [float(force_n)]
        self.control_joint_publisher.publish(msg)
        self.log_message.emit(
            f"已发送夹爪目标：{width_m * 1000.0:.1f} mm，{force_n:.1f} N"
        )
        return True

    def shutdown(self) -> None:
        self.feedback_timer.stop()
        try:
            self.executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


class StatusBadge(QLabel):
    def __init__(self, text: str = "未知") -> None:
        super().__init__(text)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumWidth(110)
        self.set_state("neutral", text)

    def set_state(self, state: str, text: str) -> None:
        colors = {
            "good": ("#eef3ef", "#34513d", "#d1dbd3"),
            "warn": ("#f5f2ea", "#695a34", "#ded6c2"),
            "bad": ("#f6eeee", "#7f3333", "#decaca"),
            "neutral": ("#f1f3f4", "#4f5962", "#d7dcdf"),
        }
        background, foreground, border = colors[state]
        self.setText(text)
        self.setStyleSheet(
            "QLabel {"
            f"background:{background}; color:{foreground}; border:1px solid {border};"
            "border-radius:5px; padding:4px 9px; font-weight:600;"
            "}"
        )


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(APP_TITLE)
        self.resize(1520, 900)
        self.setMinimumSize(1180, 720)

        self.bridge = RosBridge()
        self.last_joint_time = 0.0
        self.last_status_time = 0.0
        self.last_gripper_time = 0.0
        self.current_joints: Optional[List[float]] = None
        self.latest_status: Optional[AgxArmStatus] = None
        self.latest_gripper: Optional[GripperStatus] = None
        self.targets_initialized = False
        self.gripper_target_initialized = False
        self.enable_state: Optional[bool] = None
        self.pending_joint_target: Optional[List[float]] = None
        self.pending_gripper_target: Optional[Tuple[float, float]] = None
        self.jog_unlocked = False
        self.highlighted_joint_index: Optional[int] = None
        self._last_log_message = ""
        self._last_log_time = 0.0

        self.current_joint_labels: List[QLabel] = []
        self.motor_degree_labels: List[QLabel] = []
        self.motor_radian_labels: List[QLabel] = []
        self.motor_effort_labels: List[QLabel] = []
        self.simulation_joint_labels: List[QLabel] = []
        self.target_spinboxes: List[QDoubleSpinBox] = []
        self.calibrate_buttons: List[QPushButton] = []
        self.execute_single_buttons: List[QPushButton] = []
        self.jog_buttons: List[QPushButton] = []
        self.joint_motor_buttons: List[Tuple[QPushButton, QPushButton]] = []
        self.status_value_labels: Dict[str, QLabel] = {}

        self._build_ui()
        self._connect_signals()
        self._apply_style()

        self.health_timer = QTimer(self)
        self.health_timer.timeout.connect(self._refresh_health)
        self.health_timer.start(500)

        self.can_timer = QTimer(self)
        self.can_timer.timeout.connect(self._refresh_can)
        self.can_timer.start(2000)
        self._refresh_can()

        self._log("界面已启动；默认不会使能或移动机械臂")

    def _build_ui(self) -> None:
        root = QWidget()
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)
        root_layout.addWidget(self._build_top_bar())

        body = QWidget()
        body_layout = QHBoxLayout(body)
        body_layout.setContentsMargins(0, 0, 0, 0)
        body_layout.setSpacing(0)
        body_layout.addWidget(self._build_sidebar())

        workspace = QSplitter(Qt.Horizontal)
        workspace.setObjectName("workspaceSplitter")
        self.page_stack = QStackedWidget()
        self.page_stack.setObjectName("pageStack")
        self.page_stack.addWidget(self._build_motion_page())
        self.page_stack.addWidget(self._build_gripper_page())
        self.page_stack.addWidget(self._build_status_page())
        self.page_stack.addWidget(self._build_motor_page())
        workspace.addWidget(self.page_stack)
        workspace.addWidget(self._build_simulation_panel())
        workspace.setSizes([820, 500])
        workspace.setStretchFactor(0, 8)
        workspace.setStretchFactor(1, 5)
        body_layout.addWidget(workspace, 1)
        root_layout.addWidget(body, 1)

        self.setCentralWidget(root)
        self.statusBar().showMessage("等待 /feedback/* 数据")

    def _build_top_bar(self) -> QWidget:
        bar = QWidget()
        bar.setObjectName("topBar")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(22, 12, 18, 12)
        layout.setSpacing(10)

        brand = QLabel("PIPER X 机械臂控制台")
        brand.setObjectName("brand")
        layout.addWidget(brand)
        layout.addSpacing(12)
        layout.addWidget(QLabel("CAN"))
        self.can_badge = StatusBadge()
        layout.addWidget(self.can_badge)
        layout.addWidget(QLabel("ROS"))
        self.ros_badge = StatusBadge()
        layout.addWidget(self.ros_badge)
        layout.addWidget(QLabel("夹爪"))
        self.gripper_badge = StatusBadge()
        layout.addWidget(self.gripper_badge)
        layout.addStretch(1)
        layout.addWidget(QLabel("整机状态"))
        self.enable_badge = StatusBadge()
        layout.addWidget(self.enable_badge)

        self.enable_button = QPushButton("使能")
        self.enable_button.setObjectName("primaryButton")
        self.disable_button = QPushButton("失能")
        self.top_home_button = QPushButton("回零位")
        self.top_home_button.setObjectName("homeButton")
        self.stop_button = QPushButton("停止并保持")
        self.stop_button.setObjectName("dangerButton")
        layout.addWidget(self.enable_button)
        layout.addWidget(self.disable_button)
        layout.addWidget(self.top_home_button)
        layout.addWidget(self.stop_button)
        return bar

    def _build_sidebar(self) -> QWidget:
        sidebar = QWidget()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(160)
        layout = QVBoxLayout(sidebar)
        layout.setContentsMargins(10, 18, 10, 14)
        layout.setSpacing(8)

        section = QLabel("功能")
        section.setObjectName("sidebarSection")
        layout.addWidget(section)
        self.nav_group = QButtonGroup(self)
        self.nav_group.setExclusive(True)
        nav_items = (
            ("运动控制", 0),
            ("夹爪控制", 1),
            ("状态监控", 2),
            ("电机配置", 3),
        )
        self.nav_buttons: List[QPushButton] = []
        for title, index in nav_items:
            button = QPushButton(title)
            button.setObjectName("navButton")
            button.setCheckable(True)
            button.clicked.connect(
                lambda checked=False, page_index=index: self.page_stack.setCurrentIndex(page_index)
            )
            self.nav_group.addButton(button, index)
            self.nav_buttons.append(button)
            layout.addWidget(button)
        self.nav_buttons[0].setChecked(True)
        layout.addStretch(1)

        safety = QLabel("急停优先\n软件停止仅保持当前位置")
        safety.setObjectName("sidebarSafety")
        safety.setWordWrap(True)
        layout.addWidget(safety)
        return sidebar

    def _page_heading(self, title: str, description: str) -> QWidget:
        heading = QWidget()
        layout = QVBoxLayout(heading)
        layout.setContentsMargins(0, 0, 0, 4)
        layout.setSpacing(3)
        title_label = QLabel(title)
        title_label.setObjectName("pageTitle")
        description_label = QLabel(description)
        description_label.setObjectName("muted")
        description_label.setWordWrap(True)
        layout.addWidget(title_label)
        layout.addWidget(description_label)
        return heading

    def _build_motion_page(self) -> QWidget:
        page = QWidget()
        page.setObjectName("contentPage")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(10)
        layout.addWidget(
            self._page_heading(
                "运动控制",
                "设置关节目标并发送。",
            )
        )
        note = QLabel("示教使用实体按钮。单次关节变化不超过 20°。")
        note.setObjectName("infoNote")
        note.setWordWrap(True)
        layout.addWidget(note)
        layout.addWidget(self._build_jog_group())
        layout.addWidget(self._build_joint_group(), 1)
        return page

    def _build_status_page(self) -> QWidget:
        page = QWidget()
        page.setObjectName("contentPage")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(10)
        layout.addWidget(
            self._page_heading(
                "状态监控",
                "查看控制器状态。",
            )
        )
        layout.addWidget(self._build_status_group())
        layout.addStretch(1)
        return page

    def _build_gripper_page(self) -> QWidget:
        page = QWidget()
        page.setObjectName("contentPage")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(10)
        layout.addWidget(
            self._page_heading(
                "夹爪控制",
                "设置开度和夹持力。",
            )
        )

        warning = QLabel("首次测试使用小开度、低力度。")
        warning.setObjectName("warning")
        warning.setWordWrap(True)
        layout.addWidget(warning)

        status_group = QGroupBox("实时状态")
        status_layout = QGridLayout(status_group)
        self.gripper_status_labels: Dict[str, QLabel] = {}
        status_fields = (
            ("width", "当前开度"),
            ("force", "当前力度"),
            ("enabled", "电机状态"),
            ("homing", "归零标志"),
            ("faults", "故障状态"),
        )
        for row, (key, title) in enumerate(status_fields):
            status_layout.addWidget(QLabel(title), row, 0)
            value = QLabel("—")
            value.setObjectName("gripperStatusValue")
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.gripper_status_labels[key] = value
            status_layout.addWidget(value, row, 1)
        status_layout.setColumnStretch(1, 1)
        layout.addWidget(status_group)

        control_group = QGroupBox("目标控制")
        control_layout = QGridLayout(control_group)
        control_layout.addWidget(QLabel("目标开度"), 0, 0)
        self.gripper_width_spin = QDoubleSpinBox()
        self.gripper_width_spin.setRange(0.0, 100.0)
        self.gripper_width_spin.setDecimals(1)
        self.gripper_width_spin.setSingleStep(5.0)
        self.gripper_width_spin.setSuffix(" mm")
        self.gripper_width_spin.setEnabled(False)
        control_layout.addWidget(self.gripper_width_spin, 0, 1)

        control_layout.addWidget(QLabel("夹持力度"), 1, 0)
        self.gripper_force_spin = QDoubleSpinBox()
        self.gripper_force_spin.setRange(0.5, 3.0)
        self.gripper_force_spin.setDecimals(1)
        self.gripper_force_spin.setSingleStep(0.1)
        self.gripper_force_spin.setValue(0.5)
        self.gripper_force_spin.setSuffix(" N")
        self.gripper_force_spin.setEnabled(False)
        control_layout.addWidget(self.gripper_force_spin, 1, 1)

        self.gripper_sync_button = QPushButton("同步当前开度")
        self.gripper_send_button = QPushButton("发送目标")
        self.gripper_send_button.setObjectName("primaryButton")
        self.gripper_open_button = QPushButton("完全打开 100 mm")
        self.gripper_close_button = QPushButton("完全闭合 0 mm")
        self.gripper_close_button.setObjectName("warningButton")
        self.gripper_disable_button = QPushButton("夹爪失能")
        self.gripper_disable_button.setObjectName("dangerButton")
        for button in (
            self.gripper_sync_button,
            self.gripper_send_button,
            self.gripper_open_button,
            self.gripper_close_button,
            self.gripper_disable_button,
        ):
            button.setEnabled(False)

        controls = QHBoxLayout()
        controls.addWidget(self.gripper_sync_button)
        controls.addWidget(self.gripper_open_button)
        controls.addWidget(self.gripper_close_button)
        controls.addStretch(1)
        controls.addWidget(self.gripper_disable_button)
        controls.addWidget(self.gripper_send_button)
        control_layout.addLayout(controls, 2, 0, 1, 2)
        control_layout.setColumnStretch(1, 1)
        layout.addWidget(control_group)

        note = QLabel("夹爪操作前请先使能机械臂。")
        note.setObjectName("infoNote")
        note.setWordWrap(True)
        layout.addWidget(note)
        layout.addStretch(1)
        return page

    def _build_motor_page(self) -> QWidget:
        page = QWidget()
        page.setObjectName("contentPage")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(10)
        layout.addWidget(
            self._page_heading(
                "电机配置",
                "单关节使能与零点设置。",
            )
        )
        warning = QLabel("失能前扶稳机械臂。设零会写入控制器。")
        warning.setObjectName("warning")
        warning.setWordWrap(True)
        layout.addWidget(warning)
        layout.addWidget(self._build_motor_group(), 1)
        return page

    def _build_simulation_panel(self) -> QWidget:
        panel = QWidget()
        panel.setObjectName("simulationPanel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 10, 12, 10)
        layout.setSpacing(0)
        panel.setMinimumWidth(390)

        right_splitter = QSplitter(Qt.Vertical)
        right_splitter.setObjectName("rightSplitter")
        right_splitter.setChildrenCollapsible(False)

        preview = QWidget()
        preview.setObjectName("simulationPreview")
        preview_layout = QVBoxLayout(preview)
        preview_layout.setContentsMargins(4, 2, 4, 6)
        preview_layout.setSpacing(7)

        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title_box.setSpacing(1)
        title = QLabel("3D 预览")
        title.setObjectName("simulationTitle")
        subtitle = QLabel("实时反馈，不控制真机")
        subtitle.setObjectName("simulationSubtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box)
        header.addStretch(1)
        reset_button = QPushButton("还原视角")
        reset_button.setObjectName("viewButton")
        header.addWidget(reset_button)
        preview_layout.addLayout(header)

        telemetry = QFrame()
        telemetry.setObjectName("telemetryPanel")
        telemetry_layout = QHBoxLayout(telemetry)
        telemetry_layout.setContentsMargins(10, 7, 10, 7)
        telemetry_layout.setSpacing(12)

        tcp_grid = QGridLayout()
        tcp_grid.setHorizontalSpacing(8)
        tcp_grid.setVerticalSpacing(2)
        tcp_title = QLabel("TCP")
        tcp_title.setObjectName("telemetryTitle")
        tcp_grid.addWidget(tcp_title, 0, 0, 1, 4)
        self.tcp_labels: Dict[str, QLabel] = {}
        tcp_fields = (("x", "X", "mm"), ("y", "Y", "mm"), ("z", "Z", "mm"),
                      ("rx", "RX", "°"), ("ry", "RY", "°"), ("rz", "RZ", "°"))
        for index, (key, name, unit) in enumerate(tcp_fields):
            row = index // 3 + 1
            column = (index % 3) * 2
            tcp_grid.addWidget(QLabel(name), row, column)
            value = QLabel(f"— {unit}")
            value.setObjectName("telemetryValue")
            self.tcp_labels[key] = value
            tcp_grid.addWidget(value, row, column + 1)
        telemetry_layout.addLayout(tcp_grid, 3)

        divider = QFrame()
        divider.setFrameShape(QFrame.VLine)
        divider.setObjectName("telemetryDivider")
        telemetry_layout.addWidget(divider)

        joint_grid = QGridLayout()
        joint_grid.setHorizontalSpacing(8)
        joint_grid.setVerticalSpacing(2)
        joint_title = QLabel("关节")
        joint_title.setObjectName("telemetryTitle")
        joint_grid.addWidget(joint_title, 0, 0, 1, 6)
        for index in range(6):
            row = index // 3 + 1
            column = (index % 3) * 2
            joint_grid.addWidget(QLabel(f"J{index + 1}"), row, column)
            value = QLabel("— °")
            value.setObjectName("telemetryValue")
            self.simulation_joint_labels.append(value)
            joint_grid.addWidget(value, row, column + 1)
        telemetry_layout.addLayout(joint_grid, 2)
        preview_layout.addWidget(telemetry)

        self.arm_view = PiperSimulationWidget()
        self.arm_view.setMinimumHeight(245)
        preview_layout.addWidget(self.arm_view, 1)
        reset_button.clicked.connect(self.arm_view.reset_camera)

        hint = QLabel("拖动旋转 · 滚轮缩放")
        hint.setObjectName("simulationHint")
        hint.setAlignment(Qt.AlignCenter)
        preview_layout.addWidget(hint)

        log_group = self._build_log_group()
        log_group.setMinimumHeight(150)
        right_splitter.addWidget(preview)
        right_splitter.addWidget(log_group)
        right_splitter.setSizes([500, 210])
        right_splitter.setStretchFactor(0, 3)
        right_splitter.setStretchFactor(1, 2)
        layout.addWidget(right_splitter)
        return panel

    def _build_status_group(self) -> QGroupBox:
        group = QGroupBox("机械臂状态")
        layout = QGridLayout(group)
        fields = (
            ("ctrl_mode", "控制模式"),
            ("arm_status", "机械臂状态"),
            ("mode_feedback", "运动模式"),
            ("teach_status", "示教状态"),
            ("motion_status", "运动状态"),
            ("err_status", "错误码"),
            ("joint_limits", "角度限制"),
            ("joint_comm", "关节通信"),
        )
        for row, (key, title) in enumerate(fields):
            name_label = QLabel(title)
            value_label = QLabel("—")
            value_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.status_value_labels[key] = value_label
            layout.addWidget(name_label, row, 0)
            layout.addWidget(value_label, row, 1)
        layout.setColumnStretch(1, 1)
        return group

    def _build_joint_group(self) -> QGroupBox:
        group = QGroupBox("关节运动（单位：度）")
        layout = QVBoxLayout(group)

        self.joint_table = QTableWidget(6, 5)
        self.joint_table.setHorizontalHeaderLabels(
            [
                "关节",
                "当前角度",
                "目标角度",
                "微调目标",
                "单轴发送",
            ]
        )
        self.joint_table.verticalHeader().setVisible(False)
        self.joint_table.setAlternatingRowColors(True)
        self.joint_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.joint_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.joint_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.joint_table.cellClicked.connect(
            lambda row, column: self._highlight_joint_row(row)
        )

        for row, name in enumerate(JOINT_NAMES):
            name_item = QTableWidgetItem(name)
            name_item.setTextAlignment(Qt.AlignCenter)
            self.joint_table.setItem(row, 0, name_item)

            degree_label = QLabel("—")
            degree_label.setAlignment(Qt.AlignCenter)
            self.current_joint_labels.append(degree_label)
            self.joint_table.setCellWidget(row, 1, degree_label)

            spinbox = QDoubleSpinBox()
            lower, upper = JOINT_LIMITS_RAD[row]
            spinbox.setRange(math.degrees(lower), math.degrees(upper))
            spinbox.setDecimals(2)
            spinbox.setSingleStep(1.0)
            spinbox.setSuffix("°")
            spinbox.setEnabled(False)
            spinbox.editingFinished.connect(
                lambda index=row: self._highlight_joint_row(index)
            )
            self.target_spinboxes.append(spinbox)
            self.joint_table.setCellWidget(row, 2, spinbox)

            adjust_widget = QWidget()
            adjust_layout = QHBoxLayout(adjust_widget)
            adjust_layout.setContentsMargins(2, 1, 2, 1)
            adjust_layout.setSpacing(3)
            minus_button = QPushButton("−1°")
            plus_button = QPushButton("+1°")
            minus_button.clicked.connect(
                lambda checked=False, index=row: self._adjust_joint_target(index, -1.0)
            )
            plus_button.clicked.connect(
                lambda checked=False, index=row: self._adjust_joint_target(index, 1.0)
            )
            adjust_layout.addWidget(minus_button)
            adjust_layout.addWidget(plus_button)
            self.joint_table.setCellWidget(row, 3, adjust_widget)

            execute_button = QPushButton("发送")
            execute_button.setObjectName("primaryButton")
            execute_button.setEnabled(False)
            execute_button.clicked.connect(
                lambda checked=False, index=row: self._execute_single_joint(index)
            )
            self.execute_single_buttons.append(execute_button)
            self.joint_table.setCellWidget(row, 4, execute_button)
            self.joint_table.setRowHeight(row, 46)

        header = self.joint_table.horizontalHeader()
        header.setStretchLastSection(False)
        for column in range(self.joint_table.columnCount()):
            header.setSectionResizeMode(column, QHeaderView.Fixed)
        self.joint_table.setColumnWidth(0, 58)
        self.joint_table.setColumnWidth(1, 74)
        self.joint_table.setColumnWidth(3, 108)
        self.joint_table.setColumnWidth(4, 76)
        header.setSectionResizeMode(2, QHeaderView.Stretch)
        layout.addWidget(self.joint_table)

        controls = QHBoxLayout()
        self.home_button = QPushButton("回零位")
        self.home_button.setObjectName("warningButton")
        self.sync_targets_button = QPushButton("同步当前姿态到目标")
        self.execute_joints_button = QPushButton("发送六关节目标")
        self.execute_joints_button.setObjectName("primaryButton")
        self.execute_joints_button.setEnabled(False)
        controls.addWidget(self.home_button)
        controls.addWidget(self.sync_targets_button)
        controls.addStretch(1)
        controls.addWidget(self.execute_joints_button)
        layout.addLayout(controls)
        return group

    def _build_jog_group(self) -> QGroupBox:
        group = QGroupBox("关节点动（点击一次，移动一步）")
        layout = QVBoxLayout(group)

        settings = QHBoxLayout()
        settings.addWidget(QLabel("每步角度"))
        self.jog_step_spin = QDoubleSpinBox()
        self.jog_step_spin.setRange(0.1, 5.0)
        self.jog_step_spin.setDecimals(1)
        self.jog_step_spin.setSingleStep(0.5)
        self.jog_step_spin.setValue(1.0)
        self.jog_step_spin.setSuffix("°")
        settings.addWidget(self.jog_step_spin)
        settings.addSpacing(10)
        self.active_joint_feedback_label = QLabel("当前关节：—")
        self.active_joint_feedback_label.setObjectName("activeJointFeedback")
        settings.addWidget(self.active_joint_feedback_label)
        settings.addStretch(1)
        settings.addWidget(QLabel("点动前需先解锁"))
        self.jog_unlock_button = QPushButton("解锁点动")
        self.jog_unlock_button.setCheckable(True)
        self.jog_unlock_button.setObjectName("warningButton")
        self.jog_unlock_button.clicked.connect(self._toggle_jog_mode)
        settings.addWidget(self.jog_unlock_button)
        layout.addLayout(settings)

        controls = QGridLayout()
        controls.setHorizontalSpacing(8)
        controls.setVerticalSpacing(6)
        for index, joint_name in enumerate(JOINT_NAMES):
            row = index // 3
            column = (index % 3) * 3
            label = QLabel(joint_name)
            label.setAlignment(Qt.AlignCenter)
            minus_button = QPushButton("−")
            plus_button = QPushButton("+")
            minus_button.setEnabled(False)
            plus_button.setEnabled(False)
            minus_button.clicked.connect(
                lambda checked=False, joint=index: self._jog_joint(joint, -1.0)
            )
            plus_button.clicked.connect(
                lambda checked=False, joint=index: self._jog_joint(joint, 1.0)
            )
            self.jog_buttons.extend((minus_button, plus_button))
            controls.addWidget(label, row, column)
            controls.addWidget(minus_button, row, column + 1)
            controls.addWidget(plus_button, row, column + 2)
        layout.addLayout(controls)
        return group

    def _build_motor_group(self) -> QGroupBox:
        group = QGroupBox("六关节电机")
        layout = QVBoxLayout(group)
        self.motor_table = QTableWidget(6, 6)
        self.motor_table.setHorizontalHeaderLabels(
            ["关节", "角度", "弧度", "力矩 Nm", "电机状态", "硬件零点"]
        )
        self.motor_table.verticalHeader().setVisible(False)
        self.motor_table.setAlternatingRowColors(True)
        self.motor_table.setSelectionMode(QTableWidget.NoSelection)
        self.motor_table.setEditTriggers(QTableWidget.NoEditTriggers)

        for row, name in enumerate(JOINT_NAMES):
            name_item = QTableWidgetItem(name)
            name_item.setTextAlignment(Qt.AlignCenter)
            self.motor_table.setItem(row, 0, name_item)
            degree_label = QLabel("—")
            radian_label = QLabel("—")
            effort_label = QLabel("—")
            for label in (degree_label, radian_label, effort_label):
                label.setAlignment(Qt.AlignCenter)
            self.motor_degree_labels.append(degree_label)
            self.motor_radian_labels.append(radian_label)
            self.motor_effort_labels.append(effort_label)
            self.motor_table.setCellWidget(row, 1, degree_label)
            self.motor_table.setCellWidget(row, 2, radian_label)
            self.motor_table.setCellWidget(row, 3, effort_label)

            motor_widget = QWidget()
            motor_layout = QHBoxLayout(motor_widget)
            motor_layout.setContentsMargins(2, 1, 2, 1)
            motor_layout.setSpacing(4)
            joint_enable_button = QPushButton("使能")
            joint_enable_button.setObjectName("jointEnableButton")
            joint_disable_button = QPushButton("失能")
            joint_disable_button.setObjectName("jointDisableButton")
            joint_enable_button.clicked.connect(
                lambda checked=False, index=row + 1: self._set_joint_enabled(index, True)
            )
            joint_disable_button.clicked.connect(
                lambda checked=False, index=row + 1: self._set_joint_enabled(index, False)
            )
            motor_layout.addWidget(joint_enable_button)
            motor_layout.addWidget(joint_disable_button)
            self.joint_motor_buttons.append((joint_enable_button, joint_disable_button))
            self.motor_table.setCellWidget(row, 4, motor_widget)

            calibrate_button = QPushButton("设零")
            calibrate_button.setObjectName("calibrateButton")
            calibrate_button.setToolTip("永久将该关节当前位置写成硬件零点")
            calibrate_button.clicked.connect(
                lambda checked=False, index=row + 1: self._calibrate_joint(index)
            )
            self.calibrate_buttons.append(calibrate_button)
            self.motor_table.setCellWidget(row, 5, calibrate_button)
            self.motor_table.setRowHeight(row, 46)

        header = self.motor_table.horizontalHeader()
        for column in range(self.motor_table.columnCount()):
            header.setSectionResizeMode(column, QHeaderView.Fixed)
        self.motor_table.setColumnWidth(0, 56)
        self.motor_table.setColumnWidth(1, 75)
        self.motor_table.setColumnWidth(2, 82)
        self.motor_table.setColumnWidth(3, 74)
        self.motor_table.setColumnWidth(5, 70)
        header.setSectionResizeMode(4, QHeaderView.Stretch)
        layout.addWidget(self.motor_table)

        note = QLabel("单独使能一个关节后，如需整臂运动，请再点击顶部“使能”。")
        note.setObjectName("muted")
        layout.addWidget(note)
        return group

    def _build_log_group(self) -> QGroupBox:
        group = QGroupBox("日志")
        group.setObjectName("logGroup")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(8, 10, 8, 8)
        self.log_view = QTextEdit()
        self.log_view.setObjectName("logView")
        self.log_view.setReadOnly(True)
        self.log_view.document().setMaximumBlockCount(500)
        layout.addWidget(self.log_view)
        return group

    def _connect_signals(self) -> None:
        self.bridge.joint_received.connect(self._on_joint_received)
        self.bridge.pose_received.connect(self._on_pose_received)
        self.bridge.status_received.connect(self._on_status_received)
        self.bridge.gripper_received.connect(self._on_gripper_received)
        self.bridge.service_finished.connect(self._on_service_finished)
        self.bridge.log_message.connect(self._log)
        self.statusBar().messageChanged.connect(self._log_status_message)

        self.enable_button.clicked.connect(self._enable_arm)
        self.disable_button.clicked.connect(self._disable_arm)
        self.top_home_button.clicked.connect(self._move_home)
        self.home_button.clicked.connect(self._move_home)
        self.stop_button.clicked.connect(self._stop_arm)
        self.sync_targets_button.clicked.connect(self._sync_targets)
        self.execute_joints_button.clicked.connect(self._execute_joint_target)
        self.gripper_sync_button.clicked.connect(self._sync_gripper_target)
        self.gripper_send_button.clicked.connect(self._send_gripper_target)
        self.gripper_open_button.clicked.connect(
            lambda: self._request_gripper_preset(100.0, "完全打开")
        )
        self.gripper_close_button.clicked.connect(
            lambda: self._request_gripper_preset(0.0, "完全闭合")
        )
        self.gripper_disable_button.clicked.connect(self._disable_gripper)

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #f6f7f8; color: #22272b; }
            QWidget#topBar {
                background: #ffffff; border-bottom: 1px solid #e0e3e5;
            }
            QLabel#brand { font-size: 17px; font-weight: 700; color: #202427; }
            QWidget#sidebar {
                background: #ffffff; border-right: 1px solid #e0e3e5;
            }
            QLabel#sidebarSection {
                color: #7b8389; font-size: 9pt; font-weight: 700;
                padding: 0 10px 5px 10px;
            }
            QPushButton#navButton {
                min-height: 42px; border: 0; border-radius: 5px; padding: 0 14px;
                background: transparent; color: #50575c; text-align: left;
                font-weight: 600;
            }
            QPushButton#navButton:hover { background: #f1f3f4; }
            QPushButton#navButton:checked {
                background: #eceff1; color: #202427; border-left: 3px solid #59636a;
            }
            QLabel#sidebarSafety {
                background: #f5f6f7; color: #687076; border: 1px solid #e0e3e5;
                border-radius: 6px; padding: 10px; font-size: 9pt;
            }
            QWidget#contentPage { background: #f6f7f8; }
            QLabel#pageTitle { font-size: 19px; font-weight: 700; color: #202427; }
            QLabel#muted { color: #70787e; }
            QLabel#warning {
                background: #f6f3ec; color: #655a42; border: 1px solid #e0d9c9;
                border-radius: 5px; padding: 9px 12px;
            }
            QLabel#infoNote {
                background: #f1f3f4; color: #555e64; border: 1px solid #dfe3e5;
                border-radius: 5px; padding: 9px 11px;
            }
            QGroupBox {
                font-weight: 650; border: 1px solid #dfe3e5; border-radius: 6px;
                margin-top: 12px; padding: 12px 8px 8px 8px; background: #ffffff;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; }
            QPushButton {
                min-height: 34px; border: 1px solid #c7cdd1; border-radius: 5px;
                padding: 2px 14px; background: #ffffff;
            }
            QPushButton:hover { background: #f1f3f4; }
            QPushButton:disabled { color: #a0a6aa; background: #f0f2f3; }
            QPushButton#primaryButton { background: #3f4850; color: white; border-color: #3f4850; }
            QPushButton#primaryButton:hover { background: #30383e; }
            QPushButton#homeButton, QPushButton#warningButton {
                background: #f5f6f7; color: #30363a; border-color: #b9c0c5;
            }
            QPushButton#dangerButton { background: #8f3d3d; color: white; border-color: #823737; }
            QPushButton#dangerButton:hover { background: #793333; }
            QPushButton#calibrateButton { background: #ffffff; color: #4f585e; border-color: #bfc6ca; }
            QPushButton#calibrateButton:hover { background: #f1f3f4; }
            QPushButton#calibrateButton { min-height: 25px; padding: 1px 7px; font-size: 9pt; }
            QPushButton#jointEnableButton, QPushButton#jointDisableButton {
                min-height: 25px; padding: 1px 6px; font-size: 9pt;
            }
            QPushButton#jointEnableButton { background: #f3f5f4; color: #435149; border-color: #c8d0cb; }
            QPushButton#jointDisableButton { background: #f6f2f2; color: #754545; border-color: #d6c7c7; }
            QPushButton#viewButton { min-height: 28px; padding: 1px 12px; }
            QTableWidget { background: white; border: 1px solid #dfe3e5; gridline-color: #eceeef; }
            QTableWidget::item { padding: 4px; }
            QTableWidget::item:selected {
                background: #e5e8ea; color: #22272b; font-weight: 700;
            }
            QTableWidget QWidget[activeJointRow="true"] {
                background: #eceff1;
            }
            QTableWidget QDoubleSpinBox[activeJointRow="true"] {
                background: #ffffff; border: 2px solid #69737a;
            }
            QHeaderView::section {
                background: #f0f2f3; padding: 7px; border: 0;
                border-right: 1px solid #dfe3e5; font-weight: 650;
            }
            QDoubleSpinBox { min-height: 28px; padding: 1px 6px; }
            QTextEdit#logView {
                background: #f7f8f9; color: #4c555b; border: 1px solid #e0e3e5;
                border-radius: 4px; padding: 4px; font-family: monospace;
            }
            QWidget#simulationPanel {
                background: #ffffff; border-left: 1px solid #e0e3e5;
            }
            QWidget#simulationPreview { background: #ffffff; }
            QLabel#simulationTitle { font-size: 17px; font-weight: 700; color: #202427; }
            QLabel#simulationSubtitle, QLabel#simulationHint { color: #7b8389; font-size: 9pt; }
            QFrame#telemetryPanel {
                background: #f4f5f6; border: 1px solid #e0e3e5; border-radius: 5px;
            }
            QLabel#telemetryTitle { color: #555e64; font-weight: 700; }
            QLabel#telemetryValue {
                color: #2c3236; font-family: monospace; font-weight: 650;
                min-width: 62px;
            }
            QLabel#activeJointFeedback {
                color: #202427; font-family: monospace; font-weight: 700;
            }
            QLabel#gripperStatusValue {
                color: #2c3236; font-family: monospace; font-weight: 650;
            }
            QFrame#telemetryDivider { color: #d1d6d9; }
            QLabel#simulationFallback {
                background: #f0f2f3; color: #747c82; border: 1px solid #dfe3e5;
                font-size: 14px;
            }
            QSplitter#rightSplitter::handle { background: #eceeef; height: 5px; }
            QSplitter#workspaceSplitter::handle { background: #eceeef; width: 3px; }
            QStatusBar { background: #ffffff; border-top: 1px solid #e0e3e5; }
            """
        )

    def _feedback_fresh(self) -> bool:
        return (
            self.current_joints is not None
            and time.monotonic() - self.last_joint_time <= FEEDBACK_STALE_SECONDS
        )

    def _gripper_feedback_fresh(self) -> bool:
        return (
            self.latest_gripper is not None
            and time.monotonic() - self.last_gripper_time <= FEEDBACK_STALE_SECONDS
        )

    def _status_feedback_fresh(self) -> bool:
        return (
            self.latest_status is not None
            and time.monotonic() - self.last_status_time <= FEEDBACK_STALE_SECONDS
        )

    def _is_teaching(self) -> bool:
        if self.latest_status is None:
            return False
        return self.latest_status.teach_status in (1, 3, 4, 5)

    def _require_fresh_feedback(self, action: str) -> bool:
        if not self._feedback_fresh():
            QMessageBox.warning(
                self,
                "反馈不可用",
                f"不能{action}：没有收到新鲜的 /feedback/joint_states。\n"
                "请确认 CAN 和 agx_arm_ctrl 驱动正在运行。",
            )
            return False
        return True

    def _enable_arm(self) -> None:
        if not self._require_fresh_feedback("使能"):
            return
        self._set_arm_power_busy("enable")
        self._log("请求使能机械臂")
        self.bridge.call_set_bool("enable", "/enable_agx_arm", True)

    def _disable_arm(self) -> None:
        answer = QMessageBox.warning(
            self,
            "确认失能机械臂",
            "失能后机械臂可能失去保持力并下坠。\n\n"
            "请先扶稳机械臂、确认周围安全。是否继续失能？",
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer == QMessageBox.Yes:
            self._lock_jog()
            self._set_arm_power_busy("disable")
            self._log("确认后请求失能机械臂")
            self.bridge.call_set_bool("disable", "/enable_agx_arm", False)

    def _set_arm_power_busy(self, action: Optional[str]) -> None:
        busy = action is not None
        self.enable_button.setEnabled(not busy)
        self.disable_button.setEnabled(not busy)
        self.enable_button.setText("使能中…" if action == "enable" else "使能")
        self.disable_button.setText("失能中…" if action == "disable" else "失能")

    def _set_home_busy(self, busy: bool) -> None:
        text = "回零中…" if busy else "回零位"
        for button in (self.top_home_button, self.home_button):
            button.setEnabled(not busy)
            button.setText(text)

    def _move_home(self) -> None:
        if not self._require_fresh_feedback("回零"):
            return
        if self.enable_state is not True:
            QMessageBox.warning(self, "尚未使能", "请先通过本界面的“使能”按钮使能机械臂。")
            return
        if self._is_teaching():
            QMessageBox.warning(self, "示教模式", "机械臂仍处于示教状态，不能回零。")
            return
        answer = QMessageBox.warning(
            self,
            "确认回零位",
            "机械臂将把六个关节移动到 0 rad，这可能是一次较大的运动。\n\n"
            "请确认机械臂固定、运动范围无人无障碍，并拿好实体急停。",
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer == QMessageBox.Yes:
            self._lock_jog()
            self._set_home_busy(True)
            self._log("请求回零位")
            self.bridge.call_trigger("home", "/move_home_checked")

    def _stop_arm(self) -> None:
        if not self._require_fresh_feedback("停止"):
            return
        self._lock_jog()
        self._log("请求停止并保持当前位置")
        self.bridge.call_empty("stop", "/emergency_stop")

    def _set_gripper_controls_enabled(self, enabled: bool) -> None:
        self.gripper_width_spin.setEnabled(enabled)
        self.gripper_force_spin.setEnabled(enabled)
        for button in (
            self.gripper_sync_button,
            self.gripper_send_button,
            self.gripper_open_button,
            self.gripper_close_button,
            self.gripper_disable_button,
        ):
            button.setEnabled(enabled)

    def _sync_gripper_target(self) -> None:
        if not self._gripper_feedback_fresh() or self.latest_gripper is None:
            QMessageBox.warning(
                self,
                "夹爪反馈不可用",
                "没有收到新鲜的 /feedback/gripper_status。\n"
                "请使用 effector_type:=agx_gripper 启动驱动。",
            )
            return
        self.gripper_width_spin.setValue(float(self.latest_gripper.width) * 1000.0)
        self.gripper_target_initialized = True
        self._log("夹爪目标已同步为当前真实开度")

    def _send_gripper_target(self) -> None:
        self._request_gripper_motion(
            self.gripper_width_spin.value() / 1000.0,
            self.gripper_force_spin.value(),
            "自定义目标",
        )

    def _request_gripper_preset(self, width_mm: float, label: str) -> None:
        self.gripper_width_spin.setValue(width_mm)
        self._request_gripper_motion(
            width_mm / 1000.0,
            self.gripper_force_spin.value(),
            label,
        )

    def _request_gripper_motion(
        self, width_m: float, force_n: float, label: str
    ) -> None:
        if not self._gripper_feedback_fresh() or self.latest_gripper is None:
            QMessageBox.warning(
                self,
                "夹爪反馈不可用",
                "不能控制夹爪：没有收到新鲜的 /feedback/gripper_status。\n"
                "请确认驱动以 effector_type:=agx_gripper 启动。",
            )
            return
        if self.enable_state is not True:
            QMessageBox.warning(self, "尚未使能", "请先点击顶部“使能”。")
            return
        if self._is_teaching():
            QMessageBox.warning(self, "示教模式", "机械臂仍处于示教状态，不能控制夹爪。")
            return
        if self.pending_joint_target is not None or self.pending_gripper_target is not None:
            QMessageBox.information(self, "命令处理中", "请等待上一条控制命令完成。")
            return

        valid, reason = validate_gripper_target(width_m, force_n)
        if not valid:
            QMessageBox.warning(self, "夹爪目标无效", reason)
            return

        current_mm = float(self.latest_gripper.width) * 1000.0
        target_mm = width_m * 1000.0
        pinch_warning = (
            "\n\n完全闭合可能夹伤手指或挤压物体，请确保夹爪范围内无人。"
            if target_mm <= 0.01
            else ""
        )
        answer = QMessageBox.warning(
            self,
            "确认夹爪运动",
            f"动作：{label}\n当前开度：{current_mm:.1f} mm\n"
            f"目标开度：{target_mm:.1f} mm\n夹持力度：{force_n:.1f} N"
            f"{pinch_warning}\n\n是否发送？",
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Yes:
            return

        self.pending_gripper_target = (width_m, force_n)
        self._set_gripper_controls_enabled(False)
        self._log(f"请求夹爪运动：{target_mm:.1f} mm，{force_n:.1f} N")
        self.bridge.call_set_bool("gripper_gate_open", "/control_enable", True)

    def _disable_gripper(self) -> None:
        if not self._gripper_feedback_fresh():
            QMessageBox.warning(self, "夹爪反馈不可用", "无法确认夹爪当前状态。")
            return
        if self.pending_joint_target is not None or self.pending_gripper_target is not None:
            QMessageBox.information(self, "命令处理中", "请等待上一条控制命令完成。")
            return
        answer = QMessageBox.warning(
            self,
            "确认夹爪失能",
            "失能后夹爪将不再主动保持夹持力，物体可能掉落。\n\n"
            "请先托住物体并确认安全。是否继续？",
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Yes:
            return
        self._set_gripper_controls_enabled(False)
        self._log("确认后请求夹爪失能")
        self.bridge.call_trigger("gripper_disable", "/disable_gripper")

    def _set_joint_enabled(self, joint_index: int, enabled: bool) -> None:
        if not self._require_fresh_feedback(
            f"{'使能' if enabled else '失能'} joint{joint_index}"
        ):
            return
        if not enabled:
            answer = QMessageBox.warning(
                self,
                f"确认失能 joint{joint_index}",
                f"joint{joint_index} 失能后会失去保持力，机械臂可能下坠或转动。\n\n"
                "请先扶稳机械臂并确认周围安全。是否继续？",
                QMessageBox.Yes | QMessageBox.Cancel,
                QMessageBox.Cancel,
            )
            if answer != QMessageBox.Yes:
                return

        for enable_button, disable_button in self.joint_motor_buttons:
            enable_button.setEnabled(False)
            disable_button.setEnabled(False)
        action = "enable" if enabled else "disable"
        self._log(f"请求单独{'使能' if enabled else '失能'} joint{joint_index}")
        self.bridge.call_set_bool(
            f"joint_{action}_{joint_index}",
            f"/enable_joint_{joint_index}",
            enabled,
        )

    def _sync_targets(self) -> None:
        if not self._require_fresh_feedback("同步目标"):
            return
        assert self.current_joints is not None
        for spinbox, value in zip(self.target_spinboxes, self.current_joints):
            spinbox.setValue(math.degrees(value))
            spinbox.setEnabled(True)
        self.targets_initialized = True
        self._set_joint_motion_buttons_enabled(True)
        self._log("关节目标已同步为当前真实姿态")

    def _toggle_jog_mode(self, checked: bool) -> None:
        if not checked:
            self._lock_jog()
            self._log("关节点动已锁定")
            return
        if not self._require_fresh_feedback("解锁点动"):
            self._lock_jog()
            return
        if not self._status_feedback_fresh():
            QMessageBox.warning(self, "状态反馈不可用", "机械臂状态反馈中断，不能解锁点动。")
            self._lock_jog()
            return
        if self.enable_state is not True:
            QMessageBox.warning(self, "尚未使能", "请先通过顶部“使能”按钮使能机械臂。")
            self._lock_jog()
            return
        if self._is_teaching():
            QMessageBox.warning(self, "示教模式", "机械臂仍处于示教状态，不能解锁点动。")
            self._lock_jog()
            return

        answer = QMessageBox.warning(
            self,
            "确认解锁关节点动",
            "解锁后，点击任一关节的 +/− 按钮会立即让真机移动一步，"
            "不再逐次弹出确认框。\n\n"
            "请清空运动范围、拿好实体急停，并在操作结束后重新锁定。",
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Yes:
            self._lock_jog()
            return
        self.jog_unlocked = True
        self.jog_unlock_button.setText("锁定点动")
        self.jog_unlock_button.setObjectName("dangerButton")
        self.jog_unlock_button.style().unpolish(self.jog_unlock_button)
        self.jog_unlock_button.style().polish(self.jog_unlock_button)
        self._set_jog_buttons_enabled(True)
        self._log("关节点动已解锁；点击 +/− 将立即运动")

    def _lock_jog(self) -> None:
        self.jog_unlocked = False
        self.jog_unlock_button.setChecked(False)
        self.jog_unlock_button.setText("解锁点动")
        self.jog_unlock_button.setObjectName("warningButton")
        self.jog_unlock_button.style().unpolish(self.jog_unlock_button)
        self.jog_unlock_button.style().polish(self.jog_unlock_button)
        self._set_jog_buttons_enabled(False)

    def _set_jog_buttons_enabled(self, enabled: bool) -> None:
        for button in self.jog_buttons:
            button.setEnabled(enabled)

    def _highlight_joint_row(self, joint_index: int) -> None:
        if not 0 <= joint_index < len(JOINT_NAMES):
            return
        self.highlighted_joint_index = joint_index
        self.joint_table.selectRow(joint_index)
        for row in range(self.joint_table.rowCount()):
            active = row == joint_index
            for column in range(1, self.joint_table.columnCount()):
                widget = self.joint_table.cellWidget(row, column)
                if widget is None:
                    continue
                widget.setProperty("activeJointRow", active)
                widget.style().unpolish(widget)
                widget.style().polish(widget)
        self._update_active_joint_feedback()

    def _update_active_joint_feedback(self) -> None:
        if self.highlighted_joint_index is None:
            self.active_joint_feedback_label.setText("当前关节：—")
            return
        index = self.highlighted_joint_index
        angle = "—"
        if self.current_joints is not None:
            angle = f"{math.degrees(self.current_joints[index]):.2f}°"
        self.active_joint_feedback_label.setText(
            f"当前关节：{JOINT_NAMES[index]} · 实时 {angle}"
        )

    def _jog_joint(self, joint_index: int, direction: float) -> None:
        self._highlight_joint_row(joint_index)
        if not self.jog_unlocked:
            QMessageBox.information(self, "点动已锁定", "请先确认安全并解锁点动。")
            return
        if not self._feedback_fresh() or self.current_joints is None:
            self._lock_jog()
            QMessageBox.warning(self, "反馈中断", "关节反馈已中断，点动已自动锁定。")
            return
        if self.enable_state is not True or self._is_teaching():
            self._lock_jog()
            QMessageBox.warning(self, "状态变化", "机械臂不能继续点动，点动已自动锁定。")
            return
        if not self._status_feedback_fresh() or (
            self.latest_status is not None and self.latest_status.motion_status != 0
        ):
            self.statusBar().showMessage("机械臂尚未静止，请等待后再点动", 3000)
            return
        if self.pending_joint_target is not None or self.pending_gripper_target is not None:
            return

        step_degrees = self.jog_step_spin.value() * direction
        target = list(self.current_joints)
        target[joint_index] += math.radians(step_degrees)
        lower, upper = JOINT_LIMITS_RAD[joint_index]
        if not lower <= target[joint_index] <= upper:
            message = f"joint{joint_index + 1} 点动目标超过关节限制，已拒绝"
            self.statusBar().showMessage(message, 4000)
            self._log(message)
            return
        self._request_joint_motion(
            target,
            f"joint{joint_index + 1} 点动 {step_degrees:+.1f}°",
            require_confirmation=False,
            require_synced_target=False,
        )

    def _set_joint_motion_buttons_enabled(self, enabled: bool) -> None:
        self.execute_joints_button.setEnabled(enabled)
        for button in self.execute_single_buttons:
            button.setEnabled(enabled)

    def _invalidate_joint_targets(self) -> None:
        self._lock_jog()
        self.targets_initialized = False
        for spinbox in self.target_spinboxes:
            spinbox.setEnabled(False)
        self._set_joint_motion_buttons_enabled(False)

    def _adjust_joint_target(self, joint_index: int, delta_degrees: float) -> None:
        self._highlight_joint_row(joint_index)
        if not self.targets_initialized:
            QMessageBox.information(
                self,
                "目标尚未同步",
                "请等待反馈自动同步，或点击“同步当前姿态到目标”。",
            )
            return
        spinbox = self.target_spinboxes[joint_index]
        before = spinbox.value()
        spinbox.setValue(before + delta_degrees)
        after = spinbox.value()
        joint_name = JOINT_NAMES[joint_index]
        if math.isclose(after, before, abs_tol=1e-9):
            direction = "+" if delta_degrees > 0 else "−"
            message = f"{joint_name} 无法继续向 {direction} 调整：已到目标角度限制"
            self._log(message)
            self.statusBar().showMessage(message, 4000)
        else:
            message = f"{joint_name} 目标：{before:.2f}° → {after:.2f}°（尚未执行）"
            self._log(message)
            self.statusBar().showMessage(message, 4000)

    def _execute_single_joint(self, joint_index: int) -> None:
        self._highlight_joint_row(joint_index)
        if self.current_joints is None:
            QMessageBox.warning(self, "没有反馈", "当前没有完整关节反馈。")
            return
        target = list(self.current_joints)
        target[joint_index] = math.radians(self.target_spinboxes[joint_index].value())
        self._request_joint_motion(target, f"joint{joint_index + 1}")

    def _execute_joint_target(self) -> None:
        target = [math.radians(spinbox.value()) for spinbox in self.target_spinboxes]
        self._request_joint_motion(target, "六关节目标")

    def _request_joint_motion(
        self,
        target: Sequence[float],
        label: str,
        *,
        require_confirmation: bool = True,
        require_synced_target: bool = True,
    ) -> None:
        if not self._require_fresh_feedback("执行关节目标"):
            return
        if self.enable_state is not True:
            QMessageBox.warning(self, "尚未使能", "请先通过本界面的“使能”按钮使能机械臂。")
            return
        if self._is_teaching():
            QMessageBox.warning(self, "示教模式", "机械臂仍处于示教状态，不能执行 ROS 运动。")
            return
        if self.current_joints is None or (
            require_synced_target and not self.targets_initialized
        ):
            QMessageBox.warning(self, "目标未同步", "请先同步当前姿态到目标。")
            return

        valid, reason, max_delta = validate_joint_target(self.current_joints, target)
        if not valid:
            QMessageBox.warning(self, "目标被安全检查拒绝", reason)
            return

        if require_confirmation:
            answer = QMessageBox.question(
                self,
                "确认执行关节目标",
                f"控制对象：{label}\n本次最大关节变化：{math.degrees(max_delta):.2f}°。\n"
                "将临时打开控制通道并发送一次 MOVE J。是否继续？",
                QMessageBox.Yes | QMessageBox.Cancel,
                QMessageBox.Cancel,
            )
            if answer != QMessageBox.Yes:
                return

        self.pending_joint_target = list(target)
        self._set_joint_motion_buttons_enabled(False)
        self._set_jog_buttons_enabled(False)
        self.jog_unlock_button.setEnabled(False)
        self._log(f"请求执行：{label}")
        self.bridge.call_set_bool("joint_gate_open", "/control_enable", True)

    def _calibrate_joint(self, joint_index: int) -> None:
        if not self._require_fresh_feedback(f"标定 joint{joint_index}"):
            return
        if self.current_joints is None or self.latest_status is None:
            QMessageBox.warning(self, "状态不足", "缺少关节或机械臂状态，不能标定。")
            return
        if self.latest_status.motion_status != 0:
            QMessageBox.warning(self, "机械臂未静止", "请等待机械臂完全停止后再标定。")
            return
        if self._is_teaching():
            QMessageBox.warning(
                self,
                "仍在示教模式",
                "请先按机械臂实体按钮退出示教，确认机械臂静止后再标定。",
            )
            return

        current_rad = self.current_joints[joint_index - 1]
        current_deg = math.degrees(current_rad)
        answer = QMessageBox.critical(
            self,
            f"永久标定 joint{joint_index} 硬件零点",
            f"将把 joint{joint_index} 当前姿态写入控制器，作为新的 0°。\n"
            f"当前反馈：{current_deg:.2f}°（{current_rad:.6f} rad）\n\n"
            "整机必须保持上电，程序会自动失能这个关节。关节失能后可能突然变松或下坠，"
            "其他关节不会自动失能。\n\n"
            "请先扶稳机械臂、确认机械零位正确并拿好实体急停。该操作不是普通回零。",
            QMessageBox.Yes | QMessageBox.Cancel,
            QMessageBox.Cancel,
        )
        if answer != QMessageBox.Yes:
            return

        for button in self.calibrate_buttons:
            button.setEnabled(False)
        self.execute_joints_button.setEnabled(False)
        self._log(
            f"请求标定 joint{joint_index}：当前 {current_deg:.2f}° 将成为硬件 0°"
        )
        self.bridge.call_trigger(
            f"calibrate_joint_{joint_index}", f"/calibrate_joint_{joint_index}"
        )

    def _on_joint_received(self, msg: JointState) -> None:
        ordered = ordered_joint_values(msg)
        if ordered is None:
            self._log("收到不完整的关节反馈，已忽略")
            return
        self.current_joints = ordered
        self.last_joint_time = time.monotonic()

        effort_by_name = {
            name: float(msg.effort[index])
            for index, name in enumerate(msg.name)
            if index < len(msg.effort)
        }
        for row, value in enumerate(ordered):
            degrees = math.degrees(value)
            self.current_joint_labels[row].setText(f"{degrees:.2f}°")
            self.motor_degree_labels[row].setText(f"{degrees:.2f}°")
            self.motor_radian_labels[row].setText(f"{value:.5f}")
            effort = effort_by_name.get(JOINT_NAMES[row])
            self.motor_effort_labels[row].setText(
                "—" if effort is None else f"{effort:.3f}"
            )
            self.simulation_joint_labels[row].setText(f"{degrees:.2f}°")
        self._update_active_joint_feedback()
        self.arm_view.set_joint_positions(ordered)

        if (
            not self.targets_initialized
            and self.latest_status is not None
            and not self._is_teaching()
        ):
            for spinbox, value in zip(self.target_spinboxes, ordered):
                spinbox.setValue(math.degrees(value))
                spinbox.setEnabled(True)
            self.targets_initialized = True
            self._set_joint_motion_buttons_enabled(True)

    def _on_pose_received(self, msg: PoseStamped) -> None:
        pose = msg.pose
        roll, pitch, yaw = quaternion_to_euler_degrees(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        values = {
            "x": (pose.position.x * 1000.0, "mm"),
            "y": (pose.position.y * 1000.0, "mm"),
            "z": (pose.position.z * 1000.0, "mm"),
            "rx": (roll, "°"),
            "ry": (pitch, "°"),
            "rz": (yaw, "°"),
        }
        for key, (value, unit) in values.items():
            self.tcp_labels[key].setText(f"{value:.2f} {unit}")

    def _on_status_received(self, msg: AgxArmStatus) -> None:
        was_teaching = self._is_teaching()
        self.latest_status = msg
        self.last_status_time = time.monotonic()
        is_teaching = self._is_teaching()
        if is_teaching or (was_teaching and not is_teaching):
            self._invalidate_joint_targets()
        ctrl_mode_text = CTRL_MODE_TEXT.get(msg.ctrl_mode, "未知")
        if msg.ctrl_mode == 2 and msg.teach_status == 2:
            ctrl_mode_text = "示教已结束，等待切换 CAN"
        self.status_value_labels["ctrl_mode"].setText(
            f"{msg.ctrl_mode} · {ctrl_mode_text}"
        )
        self.status_value_labels["arm_status"].setText(
            f"{msg.arm_status} · {ARM_STATUS_TEXT.get(msg.arm_status, '未知')}"
        )
        self.status_value_labels["mode_feedback"].setText(
            f"{msg.mode_feedback} · {MOVE_MODE_TEXT.get(msg.mode_feedback, '未知')}"
        )
        self.status_value_labels["teach_status"].setText(
            f"{msg.teach_status} · {TEACH_STATUS_TEXT.get(msg.teach_status, '未知')}"
        )
        self.status_value_labels["motion_status"].setText(
            "已到达" if msg.motion_status == 0 else "运动中 / 未到达"
        )
        self.status_value_labels["err_status"].setText(str(msg.err_status))
        self.status_value_labels["joint_limits"].setText(
            "正常" if not any(msg.joint_angle_limit) else "存在超限"
        )
        self.status_value_labels["joint_comm"].setText(
            "正常" if not any(msg.communication_status_joint) else "存在通信异常"
        )

    def _on_gripper_received(self, msg: GripperStatus) -> None:
        self.latest_gripper = msg
        self.last_gripper_time = time.monotonic()
        width_m = float(msg.width)
        force_n = float(msg.force)
        self.gripper_status_labels["width"].setText(f"{width_m * 1000.0:.1f} mm")
        self.gripper_status_labels["force"].setText(f"{force_n:.2f} N")
        self.gripper_status_labels["enabled"].setText(
            "已使能" if msg.driver_enable_status else "已失能"
        )
        self.gripper_status_labels["homing"].setText(
            "已完成" if msg.homing_status else "未完成 / 未上报"
        )

        fault_names = (
            (msg.voltage_too_low, "电压过低"),
            (msg.motor_overheating, "电机过热"),
            (msg.driver_overcurrent, "驱动过流"),
            (msg.driver_overheating, "驱动过热"),
            (msg.sensor_status, "传感器异常"),
            (msg.driver_error_status, "驱动器错误"),
        )
        faults = [name for active, name in fault_names if active]
        self.gripper_status_labels["faults"].setText(
            "正常" if not faults else "、".join(faults)
        )
        if faults:
            self.gripper_badge.set_state("bad", "故障")
        elif msg.driver_enable_status:
            self.gripper_badge.set_state("good", "已使能")
        else:
            self.gripper_badge.set_state("neutral", "已失能")

        if not self.gripper_target_initialized:
            self.gripper_width_spin.setValue(width_m * 1000.0)
            self.gripper_target_initialized = True
        if self.pending_gripper_target is None:
            self._set_gripper_controls_enabled(True)
        self.arm_view.set_gripper_width(width_m)

    def _on_service_finished(self, tag: str, success: bool, message: str) -> None:
        prefix = "成功" if success else "失败"
        self._log(f"{prefix}：{message}")

        if tag in ("enable", "disable"):
            self._set_arm_power_busy(None)
        if tag == "home":
            self._set_home_busy(False)

        if tag == "enable" and success:
            self.enable_state = True
            self.enable_badge.set_state("good", "已使能")
        elif tag == "disable" and success:
            self.enable_state = False
            self.enable_badge.set_state("neutral", "已失能")
        elif tag == "home" and success:
            self.statusBar().showMessage("机械臂已完成回零", 5000)
        elif tag == "joint_gate_open":
            if success and self.pending_joint_target is not None:
                sent = self.bridge.publish_joint_target(self.pending_joint_target)
                if sent:
                    QTimer.singleShot(
                        700,
                        lambda: self.bridge.call_set_bool(
                            "joint_gate_close", "/control_enable", False
                        ),
                    )
                else:
                    self.bridge.call_set_bool(
                        "joint_gate_close", "/control_enable", False
                    )
            else:
                self.pending_joint_target = None
                self._set_joint_motion_buttons_enabled(self.targets_initialized)
                self.jog_unlock_button.setEnabled(True)
                self._set_jog_buttons_enabled(
                    self.jog_unlocked and self._feedback_fresh()
                )
        elif tag == "joint_gate_close":
            self.pending_joint_target = None
            self._set_joint_motion_buttons_enabled(self.targets_initialized)
            self.jog_unlock_button.setEnabled(True)
            self._set_jog_buttons_enabled(
                self.jog_unlocked and self._feedback_fresh()
            )
        elif tag == "gripper_gate_open":
            if success and self.pending_gripper_target is not None:
                width_m, force_n = self.pending_gripper_target
                sent = self.bridge.publish_gripper_target(width_m, force_n)
                if sent:
                    QTimer.singleShot(
                        700,
                        lambda: self.bridge.call_set_bool(
                            "gripper_gate_close", "/control_enable", False
                        ),
                    )
                else:
                    self.bridge.call_set_bool(
                        "gripper_gate_close", "/control_enable", False
                    )
            else:
                self.pending_gripper_target = None
                self._set_gripper_controls_enabled(self._gripper_feedback_fresh())
        elif tag == "gripper_gate_close":
            self.pending_gripper_target = None
            self._set_gripper_controls_enabled(self._gripper_feedback_fresh())
        elif tag == "gripper_disable":
            self._set_gripper_controls_enabled(self._gripper_feedback_fresh())
            if success:
                self.gripper_badge.set_state("neutral", "已失能")
        elif tag.startswith("calibrate_joint_"):
            joint_index = int(tag.rsplit("_", 1)[1])
            for button in self.calibrate_buttons:
                button.setEnabled(True)
            self._invalidate_joint_targets()
            self.enable_state = None
            self.enable_badge.set_state("warn", f"J{joint_index} 未使能")
            if success:
                QMessageBox.information(
                    self,
                    "硬件零点标定成功",
                    f"joint{joint_index} 的当前位置已保存为硬件 0°。\n\n"
                    "该关节仍处于失能状态。请先确认反馈接近 0，再点击“使能”，"
                    "并以低速、小角度测试。",
                )
            else:
                QMessageBox.critical(
                    self,
                    "硬件零点标定失败",
                    f"{message}\n\n请按关节可能已经失能处理，扶稳机械臂并检查状态。",
                )
        elif tag.startswith("joint_enable_") or tag.startswith("joint_disable_"):
            joint_index = int(tag.rsplit("_", 1)[1])
            for enable_button, disable_button in self.joint_motor_buttons:
                enable_button.setEnabled(True)
                disable_button.setEnabled(True)
            self._invalidate_joint_targets()
            self.enable_state = None
            if success:
                enabled = tag.startswith("joint_enable_")
                state_text = "已使能" if enabled else "已失能"
                state_style = "warn" if enabled else "bad"
                self.enable_badge.set_state(
                    state_style, f"J{joint_index} {state_text}"
                )
            else:
                self.enable_badge.set_state("warn", "关节状态未知")

        if not success:
            self.statusBar().showMessage(message, 6000)
            if tag in ("enable", "disable", "home"):
                QMessageBox.warning(self, "操作没有执行", message)

    def _refresh_can(self) -> None:
        state = read_can_state("can0")
        if state.ready:
            suffix = f" · {state.error_state}" if state.error_state else ""
            self.can_badge.set_state("good", f"UP · 1 Mbps{suffix}")
        elif not state.exists:
            self.can_badge.set_state("bad", "can0 不存在")
        elif not state.up:
            self.can_badge.set_state("bad", "can0 未启动")
        else:
            bitrate = "未知" if state.bitrate is None else str(state.bitrate)
            self.can_badge.set_state("warn", f"波特率 {bitrate}")

    def _refresh_health(self) -> None:
        now = time.monotonic()
        joint_fresh = self.current_joints is not None and now - self.last_joint_time <= 1.0
        status_fresh = self.latest_status is not None and now - self.last_status_time <= 1.0
        gripper_fresh = (
            self.latest_gripper is not None and now - self.last_gripper_time <= 1.0
        )
        if self.jog_unlocked and not (joint_fresh and status_fresh):
            self._lock_jog()

        if joint_fresh and status_fresh:
            if self.latest_status is not None and self.latest_status.arm_status != 0:
                self.ros_badge.set_state("warn", "在线 · 状态异常")
            else:
                self.ros_badge.set_state("good", "反馈在线")
            self.statusBar().showMessage("已连接 agx_arm_ctrl 反馈")
        elif joint_fresh or status_fresh:
            self.ros_badge.set_state("warn", "反馈不完整")
            self.statusBar().showMessage("只收到部分反馈话题")
        else:
            self.ros_badge.set_state("bad", "驱动离线")
            self.statusBar().showMessage("未收到反馈；请启动 agx_arm_ctrl")
            if self.jog_unlocked:
                self._lock_jog()
            if self.enable_state is True:
                self.enable_state = None
                self.enable_badge.set_state("neutral", "未知")

        if not gripper_fresh:
            self.gripper_badge.set_state("bad", "未连接")
            if self.pending_gripper_target is None:
                self._set_gripper_controls_enabled(False)

    def _log(self, message: str) -> None:
        now = time.monotonic()
        if message == self._last_log_message and now - self._last_log_time < 1.0:
            return
        self._last_log_message = message
        self._last_log_time = now
        timestamp = time.strftime("%H:%M:%S")
        self.log_view.append(f"[{timestamp}] {message}")

    def _log_status_message(self, message: str) -> None:
        if message:
            self._log(message)

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API name
        if self.pending_joint_target is not None or self.pending_gripper_target is not None:
            self.bridge.call_set_bool("shutdown_gate", "/control_enable", False)
            time.sleep(0.1)
        self.arm_view.shutdown()
        self.bridge.shutdown()
        event.accept()


def configure_palette(app: QApplication) -> None:
    app.setStyle("Fusion")
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor("#f4f6f8"))
    palette.setColor(QPalette.WindowText, QColor("#1d2731"))
    palette.setColor(QPalette.Base, QColor("#ffffff"))
    palette.setColor(QPalette.Text, QColor("#1d2731"))
    palette.setColor(QPalette.Button, QColor("#f7f9fb"))
    palette.setColor(QPalette.ButtonText, QColor("#1d2731"))
    app.setPalette(palette)
    font = QFont()
    font.setPointSize(10)
    app.setFont(font)


def main() -> int:
    smoke_test = "--smoke-test" in sys.argv
    qt_args = [arg for arg in sys.argv if arg != "--smoke-test"]
    app = QApplication(qt_args)
    app.setApplicationName(APP_TITLE)
    configure_palette(app)
    window = MainWindow()
    window.show()
    if smoke_test:
        QTimer.singleShot(1200, window.close)
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
