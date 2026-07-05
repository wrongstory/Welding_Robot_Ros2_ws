#!/usr/bin/env python3
"""FITO AMR joystick teleop GUI.

- Subscribes /joy and /cmd_vel for live monitoring.
- Publishes /teleop/soft_estop (std_msgs/Bool) on button clicks.
- Lists currently connected joysticks via /proc/bus/input/devices and /dev/input/js*.
"""
import collections
import os
import re
import signal
import subprocess
import sys
import threading
import time

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, Float64, UInt16MultiArray
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from PyQt5 import QtCore, QtWidgets, uic
from PyQt5.QtCore import QProcess
from ament_index_python.packages import get_package_share_directory


# ---- Launches managed by this GUI ------------------------------------------
# On GUI startup (or Tab 2 START ALL), the entire stack is brought up in the
# fixed order in FULL_STARTUP_SEQUENCE:
#   bridge first  → /joint_states must be flowing before downstream nodes start
#   diffdrive     → subscribes to /cmd_vel, produces motor_rpm
#   odom          → consumes /joint_states
#   wheel         → pushes wheel params to ESP32 NVS
#   motorhealth   → /joint_states staleness monitor → /motor_alive
#   teleop        → joy_linux + microntek_teleop → /cmd_vel (last so the chain
#                   is fully ready before any joystick command can flow)
TELEOP_LAUNCH_KEY = 'teleop'
MOTOR_LAUNCH_KEYS = ['bridge', 'diffdrive', 'odom',
                     'wheel', 'motorhealth']                # Tab 2 grid only
FULL_STARTUP_SEQUENCE = MOTOR_LAUNCH_KEYS + [TELEOP_LAUNCH_KEY]

# Stable USB device path for ESP32 (survives re-enumeration + USB_MODE 0/1 전환).
# /dev/csst_robot 은 시스템 udev VID/PID 기반 symlink (303a:1001).
# by-id 경로는 ARDUINO_USB_MODE 마다 descriptor prefix 달라 깨짐
# (Mode=0: Espressif_Systems_..., Mode=1: Espressif_USB_JTAG_...).
ESP32_DEVICE = '/dev/csst_robot'

LAUNCHES = [
    # (key, caption, cmd_argv, claimed_device, expected_node_name)
    #   claimed_device   : USB / serial path freed (lsof + SIGKILL holders) before start
    #   expected_node_name: ROS node name used for verification + dedup-kill
    ('bridge', 'fito_amr_serial_bridge (USB-CDC raw protocol → ESP32)',
        ['ros2', 'run', 'fito_amr_serial_bridge', 'serial_bridge_node',
         '--ros-args', '-p', f'device:={ESP32_DEVICE}', '-p', 'baud:=115200'],
        ESP32_DEVICE, 'fito_amr_serial_bridge'),
    ('diffdrive', 'diff_drive_controller',
        ['ros2', 'launch', 'fito_amr_controller', 'diff_drive_controller.launch.py'],
        None, 'diff_drive_controller'),
    ('odom', 'odometry_node',
        ['ros2', 'launch', 'fito_amr_controller', 'odometry.launch.py'],
        None, 'odometry_node'),
    ('wheel', 'wheel_params_publisher',
        ['ros2', 'launch', 'fito_amr_controller', 'wheel_params.launch.py'],
        None, 'wheel_params_publisher'),
    ('motorhealth', 'motor_health_monitor (heartbeat)',
        ['ros2', 'launch', 'fito_amr_controller', 'motor_health.launch.py'],
        None, 'motor_health_monitor'),
    ('teleop', 'microntek_teleop (joy_linux + teleop → /cmd_vel)',
        ['ros2', 'launch', 'fito_amr_teleop', 'microntek_teleop.launch.py'],
        '/dev/input/joystick_wired', 'microntek_teleop'),
]

# Expected nodes/topics queried by the "NODE CHECK" button.
# Each entry: (kind, name, min_count, description)
#   kind = 'node' | 'pub' | 'sub'
NODE_CHECK_TARGETS = [
    ('node', '/fito_amr_serial_bridge', 1, 'C++ USB-CDC bridge (replaces micro-ROS)'),
    ('node', '/joy_linux', 1, 'joystick → /joy'),
    ('node', '/microntek_teleop', 1, '/joy → /cmd_vel'),
    ('node', '/diff_drive_controller', 1, '/cmd_vel → motor commands'),
    ('node', '/odometry_node', 1, '/joint_states → /odom'),
    ('node', '/wheel_params_publisher', 1, 'wheel params → ESP32 NVS'),
    ('node', '/motor_health_monitor', 1, '/joint_states staleness → /motor_alive'),
    ('pub',  '/joy',            1, 'joy_linux'),
    ('pub',  '/joint_states',   1, 'ESP32 → host (encoder feedback)'),
    ('pub',  '/cmd_vel',        1, 'teleop → diff_drive_controller'),
    ('pub',  '/odom',           1, 'odometry_node → SLAM/nav'),
    ('pub',  '/motor_alive',    1, 'motor_health_monitor → consumers'),
    ('pub',  '/fito_amr/left_motor_rpm',  1, 'diff_drive_controller → ESP32'),
    ('pub',  '/fito_amr/right_motor_rpm', 1, 'diff_drive_controller → ESP32'),
]
# Verification timing for sequential "START ALL"
START_ALL_VERIFY_MS = 2500   # initial wait after starting before checking ROS graph
START_ALL_RETRY_MS = 2000    # additional wait for one retry


def _detect_workspace_root() -> str:
    """Best-effort detection of the colcon workspace root.

    Order: COLCON_PREFIX_PATH → joystick_gui share path → None.
    Returns absolute path if found, empty string otherwise.
    """
    p = os.environ.get('COLCON_PREFIX_PATH', '').split(':')[0]
    if p and os.path.isdir(os.path.join(p, '..', 'src')):
        return os.path.abspath(os.path.join(p, '..'))
    try:
        share = get_package_share_directory('joystick_gui')
        # share = .../install/joystick_gui/share/joystick_gui
        candidate = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
        if os.path.isdir(os.path.join(candidate, 'src')):
            return candidate
    except Exception:
        pass
    return ''


HZ_WINDOW_SEC = 1.0
JOY_TIMEOUT_SEC = 1.0


class TeleopGuiNode(Node):
    """ROS-side companion for the GUI. Stores latest data and publishes soft estop."""

    def __init__(self):
        super().__init__('joystick_gui')
        self._lock = threading.Lock()
        self._latest_cmd_vel = Twist()
        self._joy_stamps = collections.deque(maxlen=200)
        self._last_joy_time = 0.0
        self._estop_status = False
        self._deadman_active = False
        self._last_jointstate_time = 0.0  # bridge connection health (ESP32 → PC #J → /joint_states)
        # ANALOG CHECK 상태 — Tab 1 'ANALOG CHECK' 버튼이 set/reset
        self._analog_check_active = False
        self._analog_check_max_axis = 0.0
        self._analog_check_any_btn = False
        # 최신 /joy 메시지 (Tab 1 live display 용)
        self._latest_joy_msg = None

        self._estop_pub = self.create_publisher(Bool, '/teleop/soft_estop', 10)
        # Motor control publishers (for the motor_enable toggle button on Tab 1)
        self._motor_enable_pub = self.create_publisher(Bool, '/motor_enable', 10)
        self._clear_error_pub = self.create_publisher(Empty, '/clear_error', 10)
        # FREEWHEEL (protocol v4) — motor OFF + brake released. For manual
        # rotation. /motor_enable=false only cuts power; brake stuck in prior
        # state — typically engaged after any emergencyBrake. FREEWHEEL is
        # the explicit state for hand-rotation calibration / pushing.
        self._freewheel_pub = self.create_publisher(Empty, '/freewheel', 10)
        # Pipeline Health probes — track recent stamps for each link in the
        # 10-link chain. Each deque keeps the last N timestamps; Hz computed
        # over a rolling window. See
        # docs/sw-architecture/pipeline_chain_failure_modes_2026-05-08.md
        self._cmd_vel_stamps = collections.deque(maxlen=200)
        self._left_rpm_stamps = collections.deque(maxlen=200)
        self._right_rpm_stamps = collections.deque(maxlen=200)
        self._joint_states_stamps = collections.deque(maxlen=200)
        self._odom_stamps = collections.deque(maxlen=200)
        self._motor_alive_stamps = collections.deque(maxlen=200)
        # Firmware RPM limit (protocol v3): SET = #L, GET = #G, INFO = #I.
        # Wire surface defined in docs/sw-architecture/rpm_limit_2026-05-06.md §6.
        self._motor_limit_set_pub = self.create_publisher(
            UInt16MultiArray, '/fito_amr/set_motor_limit', 10)
        self._motor_limit_get_pub = self.create_publisher(
            Empty, '/fito_amr/get_motor_limit', 10)
        # motor_limit_info: bridge publishes with TRANSIENT_LOCAL so a late
        # subscriber gets the latest snapshot immediately (firmware emits #I
        # at boot, after SET, and on GET).
        limit_info_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self._latest_motor_limit_info = None  # tuple (ram_lin, ram_rot, nvs_lin, nvs_rot, source) or None
        self._motor_limit_info_seq = 0
        self.create_subscription(
            UInt16MultiArray, '/fito_amr/motor_limit_info',
            self._on_motor_limit_info, limit_info_qos)
        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Bool, '/teleop/estop_status', self._on_estop_status, 10)
        self.create_subscription(Bool, '/teleop/deadman_active', self._on_deadman_status, 10)
        # Bridge connection indicator: subscribe to /joint_states (BEST_EFFORT to match firmware QoS).
        # The badge in the UI flips based on the freshness of this stream.
        self.create_subscription(
            JointState, '/joint_states', self._on_jointstate, qos_profile_sensor_data)
        # Pipeline Health additional subscribers — see chain links L7 / L8 / L10.
        self.create_subscription(
            Float64, '/fito_amr/left_motor_rpm',
            lambda _m: self._left_rpm_stamps.append(time.monotonic()), 10)
        self.create_subscription(
            Float64, '/fito_amr/right_motor_rpm',
            lambda _m: self._right_rpm_stamps.append(time.monotonic()), 10)
        self.create_subscription(
            Odometry, '/odom',
            lambda _m: self._odom_stamps.append(time.monotonic()),
            qos_profile_sensor_data)
        self.create_subscription(
            Bool, '/motor_alive',
            lambda _m: self._motor_alive_stamps.append(time.monotonic()), 10)

    def _on_joy(self, _msg: Joy):
        now = time.monotonic()
        with self._lock:
            self._joy_stamps.append(now)
            self._last_joy_time = now
            self._latest_joy_msg = _msg  # cache for Tab 1 live display
            # ANALOG CHECK 활성 시 axes/buttons 활동 누적 (Microntek ANALOG OFF
            # 감지용 — 5s 동안 axis 변화 0 + button 0 이면 디지털 모드 = ANALOG OFF)
            if self._analog_check_active:
                if _msg.axes:
                    a_max = max(abs(a) for a in _msg.axes)
                    if a_max > self._analog_check_max_axis:
                        self._analog_check_max_axis = a_max
                if any(_msg.buttons):
                    self._analog_check_any_btn = True

    def _on_cmd_vel(self, msg: Twist):
        with self._lock:
            self._latest_cmd_vel = msg
            self._cmd_vel_stamps.append(time.monotonic())

    def _on_estop_status(self, msg: Bool):
        with self._lock:
            self._estop_status = bool(msg.data)

    def _on_deadman_status(self, msg: Bool):
        with self._lock:
            self._deadman_active = bool(msg.data)

    def _on_jointstate(self, _msg: JointState):
        # Mark wall-time of last received frame (used for bridge connection badge).
        with self._lock:
            now = time.monotonic()
            self._last_jointstate_time = now
            self._joint_states_stamps.append(now)

    def _on_motor_limit_info(self, msg: UInt16MultiArray):
        # data layout per protocol v3 §3.3: [ram_lin, ram_rot, nvs_lin, nvs_rot, source]
        d = list(msg.data)
        if len(d) < 5:
            return
        with self._lock:
            self._latest_motor_limit_info = (d[0], d[1], d[2], d[3], d[4])
            self._motor_limit_info_seq += 1

    def snapshot(self):
        now = time.monotonic()
        with self._lock:
            cv = self._latest_cmd_vel
            stamps = list(self._joy_stamps)
            last_joy = self._last_joy_time
            estop = self._estop_status
            deadman = self._deadman_active
            last_js = self._last_jointstate_time
            limit_info = self._latest_motor_limit_info
            limit_seq = self._motor_limit_info_seq
        recent = [t for t in stamps if now - t <= HZ_WINDOW_SEC]
        joy_hz = len(recent) / HZ_WINDOW_SEC if recent else 0.0
        joy_alive = (now - last_joy) <= JOY_TIMEOUT_SEC if last_joy > 0 else False
        # Bridge connection state — 3-tier:
        #   'connected'    : last #J within 200 ms
        #   'reconnecting' : last #J 0.2s..2s ago (during USB-CDC re-enum / momentary stall)
        #   'down'         : never seen, or last #J > 2 s ago
        if last_js <= 0:
            bridge_state = 'down'
        else:
            age = now - last_js
            if age <= 0.2:
                bridge_state = 'connected'
            elif age <= 2.0:
                bridge_state = 'reconnecting'
            else:
                bridge_state = 'down'
        return {
            'linear': float(cv.linear.x),
            'angular': float(cv.angular.z),
            'joy_hz': joy_hz,
            'joy_alive': joy_alive,
            'estop_latched': estop,
            'deadman_active': deadman,
            'bridge_state': bridge_state,
            'motor_limit_info': limit_info,    # None or (ram_lin, ram_rot, nvs_lin, nvs_rot, source)
            'motor_limit_seq': limit_seq,
        }

    def publish_estop(self, latch: bool):
        m = Bool()
        m.data = bool(latch)
        self._estop_pub.publish(m)

    def publish_motor_enable(self, enable: bool):
        """Publish a /motor_enable Bool. On enable=True, also publishes /clear_error
        a couple of times so the firmware exits any latched FAULT state."""
        if enable:
            em = Empty()
            for _ in range(3):
                self._clear_error_pub.publish(em)
        m = Bool(); m.data = bool(enable)
        # Send a few copies for reliability since #E is event-driven (no retransmit).
        for _ in range(5):
            self._motor_enable_pub.publish(m)

    def start_analog_check(self):
        """5s 동안 axes/buttons 활동을 trace. Tab 1 ANALOG CHECK 버튼이 호출."""
        with self._lock:
            self._analog_check_max_axis = 0.0
            self._analog_check_any_btn = False
            self._analog_check_active = True

    def stop_analog_check(self):
        """Trace 종료 + 결과 dict 반환."""
        with self._lock:
            self._analog_check_active = False
            return {
                'max_axis': self._analog_check_max_axis,
                'any_btn': self._analog_check_any_btn,
            }

    def latest_joy(self):
        """최신 /joy 메시지의 (axes, buttons) tuple snapshot. 없으면 None."""
        with self._lock:
            if self._latest_joy_msg is None:
                return None
            return (tuple(self._latest_joy_msg.axes),
                    tuple(self._latest_joy_msg.buttons))

    def publish_freewheel(self):
        """Send /freewheel (Empty) → bridge encodes #F → firmware:
        motor power OFF + brake RELEASED. Distinct from /motor_enable=false
        which only cuts motor power. Used for manual rotation (kinematic
        calibration, hand-pushing). Firmware ACKs with #A type='F'."""
        em = Empty()
        # Send a few copies for reliability since #F is event-driven (no retransmit).
        for _ in range(3):
            self._freewheel_pub.publish(em)

    def publish_motor_limit_set(self, max_lin_rpm: int, max_rot_rpm: int, persist: bool):
        """Send #L (LIMIT_SET) via the bridge. Range 1..3000 (firmware sanitizes
        out-of-range with NACK_RANGE). persist=True writes to NVS, False = RAM only."""
        m = UInt16MultiArray()
        m.data = [int(max_lin_rpm), int(max_rot_rpm), 1 if persist else 0]
        self._motor_limit_set_pub.publish(m)

    def publish_motor_limit_get(self):
        """Send #G (LIMIT_GET). Firmware replies with #I → /fito_amr/motor_limit_info."""
        self._motor_limit_get_pub.publish(Empty())

    # ----- Pipeline Health probe helpers ----------------------------------
    @staticmethod
    def _hz_and_age(stamps: collections.deque, now: float, window: float = 1.0):
        """Return (hz, last_age_sec) from a stamp deque. hz computed over
        rolling `window` seconds; last_age is now - last stamp (None if empty)."""
        if not stamps:
            return 0.0, None
        last = stamps[-1]
        recent = [t for t in stamps if now - t <= window]
        hz = len(recent) / window if recent else 0.0
        return hz, now - last

    def pipeline_snapshot(self):
        """Build a chain-wide health snapshot for the Pipeline Health tab.
        Returns dict {link_name: (hz, age_sec_or_None)}. Caller decides
        green/yellow/red threshold + aggregate status."""
        now = time.monotonic()
        with self._lock:
            joy_stamps = list(self._joy_stamps)
            cmd_stamps = list(self._cmd_vel_stamps)
            lrpm_stamps = list(self._left_rpm_stamps)
            rrpm_stamps = list(self._right_rpm_stamps)
            js_stamps = list(self._joint_states_stamps)
            odom_stamps = list(self._odom_stamps)
            motor_stamps = list(self._motor_alive_stamps)
        return {
            '/joy': self._hz_and_age(joy_stamps, now),
            '/cmd_vel': self._hz_and_age(cmd_stamps, now),
            '/fito_amr/left_motor_rpm': self._hz_and_age(lrpm_stamps, now),
            '/fito_amr/right_motor_rpm': self._hz_and_age(rrpm_stamps, now),
            '/joint_states': self._hz_and_age(js_stamps, now),
            '/odom': self._hz_and_age(odom_stamps, now),
            '/motor_alive': self._hz_and_age(motor_stamps, now),
        }

    # ------------------------------------------------------------- parameter ops
    # Used by the Settings tab to read/write velocity limits on the active
    # teleop node (microntek_teleop or pro_controller_teleop). Uses the standard
    # rcl_interfaces SetParameters/GetParameters services every rclpy node exposes.

    @staticmethod
    def _wait_future(future, timeout_sec: float):
        """Poll a future until done (the main rclpy.spin thread services it)."""
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.done()

    def get_double_params(self, target_node: str, names: list, timeout_sec: float = 1.5):
        """Read DOUBLE-valued parameters from `target_node`. Returns dict
        {name: float} for every successfully-read name. Missing keys mean the
        target node was not reachable or the parameter is not declared.
        The returned dict is empty if the service is not reachable."""
        cli = self.create_client(GetParameters, f'{target_node}/get_parameters')
        try:
            if not cli.wait_for_service(timeout_sec=timeout_sec):
                return {}
            req = GetParameters.Request()
            req.names = list(names)
            future = cli.call_async(req)
            if not self._wait_future(future, timeout_sec):
                return {}
            res = future.result()
            if res is None:
                return {}
            out = {}
            for name, pv in zip(names, res.values):
                if pv.type == ParameterType.PARAMETER_DOUBLE:
                    out[name] = float(pv.double_value)
            return out
        finally:
            self.destroy_client(cli)

    def set_double_params(self, target_node: str, kv: dict, timeout_sec: float = 1.5):
        """Set DOUBLE-valued parameters on `target_node`. Returns
        (ok: bool, msg: str). ok=True iff every value was accepted."""
        cli = self.create_client(SetParameters, f'{target_node}/set_parameters')
        try:
            if not cli.wait_for_service(timeout_sec=timeout_sec):
                return False, f'service {target_node}/set_parameters unavailable'
            req = SetParameters.Request()
            params = []
            for name, val in kv.items():
                p = Parameter()
                p.name = name
                p.value.type = ParameterType.PARAMETER_DOUBLE
                p.value.double_value = float(val)
                params.append(p)
            req.parameters = params
            future = cli.call_async(req)
            if not self._wait_future(future, timeout_sec):
                return False, 'set_parameters call timed out'
            res = future.result()
            if res is None:
                return False, 'set_parameters returned None'
            bad = [(n, r.reason) for n, r in zip(kv.keys(), res.results) if not r.successful]
            if bad:
                return False, '; '.join(f'{n}: {r}' for n, r in bad)
            return True, 'ok'
        finally:
            self.destroy_client(cli)


# --------------------------------------------------------------------- joystick scan
_DEVICES_FILE = '/proc/bus/input/devices'


def scan_joysticks():
    """Return a list of dicts describing connected joysticks.

    Each dict: {path, name, vid, pid, bustype, bus_label, sysfs}
    """
    if not os.path.exists(_DEVICES_FILE):
        return []
    try:
        with open(_DEVICES_FILE, 'r') as f:
            blob = f.read()
    except Exception:
        return []

    results = []
    blocks = blob.split('\n\n')
    for blk in blocks:
        if 'Handlers=' not in blk or 'js' not in blk:
            continue
        info = {'name': '', 'vid': '', 'pid': '', 'bustype': '', 'sysfs': '', 'js_node': ''}
        for line in blk.splitlines():
            if line.startswith('I:'):
                m = re.search(r'Bus=([0-9a-fA-F]+)', line)
                if m:
                    info['bustype'] = m.group(1)
                m = re.search(r'Vendor=([0-9a-fA-F]+)', line)
                if m:
                    info['vid'] = m.group(1).lower()
                m = re.search(r'Product=([0-9a-fA-F]+)', line)
                if m:
                    info['pid'] = m.group(1).lower()
            elif line.startswith('N:'):
                m = re.search(r'Name="([^"]*)"', line)
                if m:
                    info['name'] = m.group(1).strip()
            elif line.startswith('S:'):
                m = re.search(r'Sysfs=(\S+)', line)
                if m:
                    info['sysfs'] = m.group(1)
            elif line.startswith('H:'):
                m = re.search(r'(js\d+)', line)
                if m:
                    info['js_node'] = m.group(1)
        if not info['js_node']:
            continue
        # USB = 0003, Bluetooth = 0005
        bus_label = {'0003': 'USB', '0005': 'BT'}.get(info['bustype'], f"BUS{info['bustype']}")
        info['bus_label'] = bus_label
        info['path'] = f"/dev/input/{info['js_node']}"
        results.append(info)
    return results


def find_symlinks_for(path: str):
    """Return udev symlinks pointing at `path` under /dev/input/."""
    base = '/dev/input'
    target = os.path.realpath(path)
    links = []
    try:
        for name in os.listdir(base):
            full = os.path.join(base, name)
            if os.path.islink(full) and os.path.realpath(full) == target:
                links.append(full)
    except OSError:
        pass
    return links


# --------------------------------------------------------------------- Qt panel
class TeleopPanel(QtWidgets.QWidget):
    def __init__(self, ros_node: TeleopGuiNode, ui_path: str):
        super().__init__()
        uic.loadUi(ui_path, self)
        self.ros_node = ros_node
        self._estop_latched = False
        self._selected_path = None
        self._applied_path = None
        self._prev_joy_alive = None  # track transitions only

        ros_node.get_logger().info(f'GUI started — UI: {ui_path}')
        ros_node.get_logger().info(
            'topics: subscribing /joy + /cmd_vel, publishing /teleop/soft_estop')

        self.btn_soft_estop.clicked.connect(self._on_soft_estop)
        self.btn_clear_latch.clicked.connect(self._on_clear_latch)
        self.btn_scan_devices.clicked.connect(self._on_scan)
        self.btn_apply_device.clicked.connect(self._on_apply)
        if hasattr(self, 'btn_analog_check'):
            self.btn_analog_check.clicked.connect(self._on_analog_check_clicked)
        self.lst_devices.itemSelectionChanged.connect(self._on_device_selection)

        # ---- Tab 2: Motor Launch wiring ----
        self._procs = {}  # key -> QProcess (or None)
        for key, _caption, _cmd, _dev, _node in LAUNCHES:
            self._procs[key] = None
            start_btn = getattr(self, f'btn_start_{key}', None)
            stop_btn = getattr(self, f'btn_stop_{key}', None)
            if start_btn is not None:
                start_btn.clicked.connect(lambda _checked=False, k=key: self._start_launch(k))
            if stop_btn is not None:
                stop_btn.clicked.connect(lambda _checked=False, k=key: self._stop_launch(k))
        self.btn_start_all.clicked.connect(self._start_all_launches)
        self.btn_stop_all.clicked.connect(self._stop_all_launches)
        self.btn_node_check.clicked.connect(self._on_node_check)
        self.btn_clear_log.clicked.connect(lambda: self.txt_log.clear())
        # Motor enable toggle (publishes /motor_enable, optionally /clear_error)
        if hasattr(self, 'btn_motor_enable'):
            self.btn_motor_enable.toggled.connect(self._on_motor_enable_toggled)
        # FREEWHEEL button — motor OFF + brake RELEASED (protocol v4 #F).
        if hasattr(self, 'btn_freewheel'):
            self.btn_freewheel.clicked.connect(self._on_freewheel_clicked)

        # ---- Tab 3: Settings wiring (velocity limits) ----
        if hasattr(self, 'btn_settings_apply'):
            self.btn_settings_apply.clicked.connect(self._on_settings_apply)
        if hasattr(self, 'btn_settings_refresh'):
            self.btn_settings_refresh.clicked.connect(self._on_settings_refresh)
        if hasattr(self, 'btn_settings_defaults'):
            self.btn_settings_defaults.clicked.connect(self._on_settings_defaults)

        # ---- Tab 3: Firmware RPM limits wiring (protocol v3 #L/#G/#I) ----
        if hasattr(self, 'btn_fw_apply_ram'):
            self.btn_fw_apply_ram.clicked.connect(lambda: self._on_fw_apply(persist=False))
        if hasattr(self, 'btn_fw_apply_nvs'):
            self.btn_fw_apply_nvs.clicked.connect(lambda: self._on_fw_apply(persist=True))
        if hasattr(self, 'btn_fw_refresh'):
            self.btn_fw_refresh.clicked.connect(self._on_fw_refresh)
        if hasattr(self, 'btn_fw_load_nvs'):
            self.btn_fw_load_nvs.clicked.connect(self._on_fw_load_nvs)
        self._last_fw_seq_seen = 0
        self._load_nvs_pending = False

        # ---- Tab 4: Pipeline Health ----
        if hasattr(self, 'tbl_health'):
            self._init_health_table()
        # Wheel kinematics for m/s↔RPM and deg/s↔RPM conversion. Initially
        # populated from the values diff_drive_controller declares (matches
        # diff_drive_params.yaml — wheel_radius=0.085 m, wheel_base=0.330 m).
        # Refreshed via _refresh_wheel_kinematics() on sequence-complete.
        self._wheel_radius = 0.085
        self._wheel_base = 0.330

        # The auto-start sequence takes ~14-18s to reach the teleop node, so
        # the initial REFRESH must wait until after that. Hooked from
        # _start_all_step's completion path — no fixed timer needed.

        self._start_all_index = 0

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(50)  # 20 Hz

        self._on_scan()  # initial scan
        # Auto-start the FULL motor + teleop stack on GUI launch, so a single
        # `ros2 run joystick_gui gui_node` brings up everything needed to drive:
        # bridge → diff_drive_controller → odometry → wheel_params → motor_health
        # → joy_linux + microntek_teleop. Equivalent to clicking Tab 2 START ALL
        # plus Tab 1 auto-teleop. User just has to press MOTOR ENABLE + deadman.
        QtCore.QTimer.singleShot(500, self._start_all_launches)

    # --------- handlers
    def _on_soft_estop(self):
        # Banner state will follow teleop's /teleop/estop_status; just send the request.
        self.ros_node.publish_estop(True)
        self.ros_node.get_logger().warn('GUI soft E-STOP request sent')

    def _on_clear_latch(self):
        self.ros_node.publish_estop(False)
        self.ros_node.get_logger().info('GUI clear-latch request sent')

    def _on_scan(self):
        log = self.ros_node.get_logger()
        log.info('SCAN: enumerating joysticks via /proc/bus/input/devices')
        self.lst_devices.clear()
        devices = scan_joysticks()
        if not devices:
            item = QtWidgets.QListWidgetItem('(no joysticks found)')
            item.setFlags(QtCore.Qt.NoItemFlags)
            self.lst_devices.addItem(item)
            self.lbl_conn_status.setText('●  scanned: 0 device(s)')
            log.warn('SCAN: 0 joysticks found')
            return
        log.info(f'SCAN: found {len(devices)} joystick(s)')
        for i, d in enumerate(devices):
            symlinks = find_symlinks_for(d['path'])
            sym = symlinks[0] if symlinks else d['path']
            short_path = sym.replace('/dev/input/', '')
            short_name = ' '.join(d['name'].split())  # collapse multiple spaces
            label = f"[{d['bus_label']}] {short_path} — {short_name}"
            tooltip = (f"Path:    {d['path']}\n"
                       f"Symlink: {sym}\n"
                       f"Name:    {short_name}\n"
                       f"VID:PID: {d['vid']}:{d['pid']}\n"
                       f"Bus:     {d['bus_label']}")
            item = QtWidgets.QListWidgetItem(label)
            item.setToolTip(tooltip)
            item.setData(QtCore.Qt.UserRole, sym)
            self.lst_devices.addItem(item)
            log.info(f"  [{i}] {d['bus_label']:3s}  {sym}  ({d['vid']}:{d['pid']})  {short_name}")
        self.lbl_conn_status.setText(f"●  scanned: {len(devices)} device(s)")

    def _on_device_selection(self):
        items = self.lst_devices.selectedItems()
        if not items:
            self._selected_path = None
            return
        path = items[0].data(QtCore.Qt.UserRole)
        self._selected_path = path

    def _on_apply(self):
        if not self._selected_path:
            return
        self._applied_path = self._selected_path
        self.ros_node.get_logger().info(
            f'Applied joystick selection: {self._applied_path} '
            f'(restart teleop launch with joy_dev:={self._applied_path} to take effect)')

    def _on_analog_check_clicked(self):
        """ANALOG CHECK — 5s 동안 axes/buttons 활동 측정 → ANALOG ON/OFF 판정.

        안전: 클릭 즉시 /motor_enable=false 발행 + UI 토글 OFF 동기화 (사용자가
        check 중에 stick 움직일 때 모터가 의도치 않게 회전 안 하도록).
        Microntek 컨트롤러의 ANALOG 토글 OFF 상태에서 운전 시도 시 cmd_vel
        가 0 으로 나가는 사고 ([memory:microntek_joystick_analog_toggle.md]) 방지.
        """
        # 1. 안전: motor disable
        self.ros_node.publish_motor_enable(False)
        if hasattr(self, 'btn_motor_enable') and self.btn_motor_enable.isChecked():
            self.btn_motor_enable.blockSignals(True)
            self.btn_motor_enable.setChecked(False)
            self.btn_motor_enable.setText('MOTORS DISABLED')
            self.btn_motor_enable.blockSignals(False)
        # 2. 버튼 비활성화 (5s 동안 중복 click 방지)
        self.btn_analog_check.setEnabled(False)
        self.btn_analog_check.setText('CHECKING... 5s')
        # 3. trace 시작
        self.ros_node.start_analog_check()
        self._append_log('[analog-check] motors DISABLED for safety. '
                         '5s 동안 모든 스틱 + 버튼 움직여 주세요.')
        # 4. 5s 후 결과 확인
        QtCore.QTimer.singleShot(5000, self._on_analog_check_done)

    def _on_analog_check_done(self):
        r = self.ros_node.stop_analog_check()
        max_axis = r['max_axis']
        any_btn = r['any_btn']
        # Threshold: axis > 0.1 (joy_linux deadzone 0.05 + 노이즈 마진) 또는 button pressed
        analog_on = (max_axis > 0.1) or any_btn
        # Restore button
        self.btn_analog_check.setEnabled(True)
        self.btn_analog_check.setText('ANALOG CHECK')
        if analog_on:
            self._append_log(
                f'[analog-check] ✓ ANALOG MODE ON  max_axis={max_axis:.2f}  '
                f'any_btn={any_btn}  — 스틱/버튼 정상 입력 감지')
        else:
            self._append_log(
                f'[analog-check] ✗ ANALOG MODE OFF  max_axis={max_axis:.2f}  '
                f'any_btn={any_btn}  — 컨트롤러 본체의 ANALOG 버튼을 눌러 LED 점등 후 다시 확인')

    # --------- motor launch management (Tab 2)

    @staticmethod
    def _spec_for(key):
        """Return (cmd_argv, claimed_device, expected_node) tuple for `key`."""
        for k, _cap, cmd, dev, node in LAUNCHES:
            if k == key:
                return cmd, dev, node
        return None, None, None

    def _cleanup_device(self, dev_path: str) -> int:
        """SIGKILL all processes holding `dev_path`. Returns count killed."""
        if not dev_path or not os.path.exists(dev_path):
            return 0
        try:
            r = subprocess.run(['lsof', '-t', dev_path],
                               capture_output=True, text=True, timeout=2)
            pids = [int(p) for p in r.stdout.strip().split('\n')
                    if p.strip().isdigit()]
        except Exception:
            return 0
        killed = 0
        for pid in pids:
            if pid == os.getpid():
                continue
            try:
                os.kill(pid, signal.SIGKILL)
                killed += 1
            except (ProcessLookupError, PermissionError):
                pass
        return killed

    def _kill_processes_matching(self, needle: str) -> int:
        """SIGKILL processes whose /proc/<pid>/cmdline contains `needle`."""
        if not needle:
            return 0
        killed = 0
        my_pid = os.getpid()
        try:
            entries = os.listdir('/proc')
        except OSError:
            return 0
        for name in entries:
            if not name.isdigit():
                continue
            pid = int(name)
            if pid == my_pid:
                continue
            try:
                with open(f'/proc/{pid}/cmdline', 'rb') as f:
                    cmdline = f.read().decode('utf-8', errors='ignore').replace('\x00', ' ')
            except (OSError, IOError):
                continue
            if needle in cmdline:
                try:
                    os.kill(pid, signal.SIGKILL)
                    killed += 1
                except (ProcessLookupError, PermissionError):
                    pass
        return killed

    def _start_launch(self, key: str):
        # If our own QProcess is still running, restart cleanly.
        proc = self._procs.get(key)
        if proc is not None and proc.state() != QProcess.NotRunning:
            self._append_log(f'[{key}] already running locally — restarting')
            self._stop_launch(key)

        cmd_argv, claimed_device, expected_node = self._spec_for(key)
        if cmd_argv is None:
            return

        # Pre-flight 0: ALWAYS kill any process running the same executable.
        # For `ros2 run pkg exec`, executable name is argv[3].
        # For `ros2 launch pkg launch_file`, the launch file name is argv[3].
        # This catches stale processes that don't hold the claimed device
        # (zombie agents, leftover launches from prior runs).
        if len(cmd_argv) >= 4 and cmd_argv[0] == 'ros2':
            exe_match = cmd_argv[3]
            n = self._kill_processes_matching(exe_match)
            if n > 0:
                self._append_log(
                    f'[{key}] killed {n} pre-existing process(es) matching "{exe_match}"')

        # Pre-flight 1: free any USB/serial device we are about to claim.
        if claimed_device:
            n = self._cleanup_device(claimed_device)
            if n > 0:
                self._append_log(
                    f'[{key}] freed {claimed_device} (killed {n} holder PID(s))')

        # Pre-flight 2: kill any external process already running this node.
        if expected_node:
            n = self._kill_processes_matching(expected_node)
            if n > 0:
                self._append_log(
                    f'[{key}] killed {n} stale process(es) matching "{expected_node}"')

        # Spawn fresh QProcess.
        proc = QProcess(self)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        ws = _detect_workspace_root()
        if ws:
            proc.setWorkingDirectory(ws)
        proc.readyReadStandardOutput.connect(lambda k=key: self._on_launch_output(k))
        proc.finished.connect(lambda _ec, _es, k=key: self._on_launch_finished(k))
        self._procs[key] = proc
        cmd, *args = cmd_argv
        cwd = ws or os.getcwd()
        self._append_log(f'[{key}] cwd={cwd}')
        self._append_log(f'[{key}] starting: {" ".join(cmd_argv)}')
        proc.start(cmd, args)
        self._set_status_dot(key, True)

    def _stop_launch(self, key: str):
        proc = self._procs.get(key)
        if proc is not None and proc.state() != QProcess.NotRunning:
            self._append_log(f'[{key}] terminating ...')
            proc.terminate()
            if not proc.waitForFinished(5000):
                self._append_log(f'[{key}] SIGTERM timed out, killing')
                proc.kill()
                proc.waitForFinished(2000)
        # Also nuke any external process that might still be running the same node.
        _cmd, dev, expected_node = self._spec_for(key)
        if expected_node:
            n = self._kill_processes_matching(expected_node)
            if n > 0:
                self._append_log(
                    f'[{key}] killed {n} external process(es) matching "{expected_node}"')

    def _stop_all_launches(self):
        for key in list(self._procs.keys()):
            self._stop_launch(key)
        # Belt-and-braces: sweep any leftover processes that match our managed
        # launches even if QProcess didn't track them (external runs).
        sweep_targets = []
        for key, _cap, _cmd, _dev, expected_node in LAUNCHES:
            if expected_node:
                sweep_targets.append(expected_node)
        # Legacy sweep: kill any leftover micro_ros_agent if user transitioned
        # from old micro-ROS-based launch (the new bridge owns serial now).
        sweep_targets.append('micro_ros_agent')
        sweep_targets.append('serial_bridge_node')
        total = 0
        for needle in sweep_targets:
            total += self._kill_processes_matching(needle)
        if total > 0:
            self._append_log(f'[stop-all] swept {total} stale process(es)')

    def _start_all_launches(self):
        # Sequential bring-up: each launch must be verified before the next
        # one starts. fito_amr_serial_bridge first so /joint_states is flowing
        # before downstream nodes look for it.
        self._start_all_index = 0
        self._append_log('[start-all] starting verified sequence ...')
        self._start_all_step()

    def _start_all_step(self):
        # START ALL = full motor + teleop stack in FULL_STARTUP_SEQUENCE order.
        if self._start_all_index >= len(FULL_STARTUP_SEQUENCE):
            self._append_log('[start-all] sequence complete ✓')
            # Settings-tab refresh: teleop now reachable, firmware bridge up.
            QtCore.QTimer.singleShot(500, self._on_settings_refresh)
            QtCore.QTimer.singleShot(700, self._refresh_wheel_kinematics)
            QtCore.QTimer.singleShot(900, self._on_fw_refresh)
            return
        idx = self._start_all_index
        key = FULL_STARTUP_SEQUENCE[idx]
        self._start_all_index += 1
        self._start_launch(key)
        # Verify after grace period (retry once if first check fails).
        QtCore.QTimer.singleShot(
            START_ALL_VERIFY_MS, lambda k=key: self._verify_then_next(k, retry=1))

    def _verify_then_next(self, key: str, retry: int):
        _cmd, _dev, expected_node = self._spec_for(key)
        proc = self._procs.get(key)
        proc_alive = proc is not None and proc.state() == QProcess.Running

        if not proc_alive:
            self._append_log(
                f'[{key}] ✗ process died — aborting START ALL sequence')
            return

        if expected_node:
            try:
                nodes = set(self.ros_node.get_node_names())
            except Exception as e:
                self._append_log(f'[{key}] verify err: {e}')
                nodes = set()
            if expected_node in nodes:
                self._append_log(f'[{key}] ✓ node "{expected_node}" up — proceeding')
                QtCore.QTimer.singleShot(300, self._start_all_step)
                return
            if retry > 0:
                self._append_log(f'[{key}] node not yet seen, retrying ...')
                QtCore.QTimer.singleShot(
                    START_ALL_RETRY_MS,
                    lambda k=key: self._verify_then_next(k, retry - 1))
                return
            self._append_log(
                f'[{key}] ✗ node "{expected_node}" never appeared — aborting sequence')
            return

        # No expected node defined. Process-alive was enough.
        self._append_log(f'[{key}] ✓ process alive — proceeding')
        QtCore.QTimer.singleShot(300, self._start_all_step)

    def _on_node_check(self):
        """Verify expected nodes/topics are present. Output to log."""
        log = self._append_log
        log('=' * 64)
        log('[node-check] querying ROS graph ...')
        try:
            node_names = set(self.ros_node.get_node_names())
        except Exception as e:
            log(f'[node-check] ERR get_node_names: {e}')
            return
        ok_count = fail_count = 0
        # Build {expected_node_name -> launch key} so we can light up status
        # dots for nodes that were started outside the GUI.
        node_to_key = {n: k for k, _c, _cmd, _dev, n in LAUNCHES if n}
        # The new C++ bridge node is fito_amr_serial_bridge — already in
        # node_to_key via the LAUNCHES table. Old amr_motor_ctrl alias kept
        # for backwards-compat: if old micro-ROS firmware is still flashed
        # the GUI should still light up the bridge dot when /amr_motor_ctrl
        # appears (so user gets correct visual feedback during migration).
        node_to_key['amr_motor_ctrl'] = 'bridge'
        for kind, name, min_count, desc in NODE_CHECK_TARGETS:
            try:
                if kind == 'node':
                    bare = name.lstrip('/')
                    present = bare in node_names or name in node_names
                    count = 1 if present else 0
                    # Sync Tab 2 status dot for externally-started nodes too.
                    launch_key = node_to_key.get(bare)
                    if launch_key is not None:
                        self._set_status_dot(launch_key, present)
                elif kind == 'pub':
                    infos = self.ros_node.get_publishers_info_by_topic(name)
                    count = len(infos)
                elif kind == 'sub':
                    infos = self.ros_node.get_subscriptions_info_by_topic(name)
                    count = len(infos)
                else:
                    log(f'[node-check] unknown kind: {kind}')
                    continue
                ok = count >= min_count
                mark = 'OK  ' if ok else 'FAIL'
                if ok:
                    ok_count += 1
                else:
                    fail_count += 1
                tag = f'{kind:3s}'
                cnt = f'(count={count})'
                log(f'  [{mark}] {tag} {name:35s} {cnt:12s}  {desc}')
            except Exception as e:
                fail_count += 1
                log(f'  [ERR ] {kind:3s} {name:35s}              {e}')
        log('-' * 64)
        log(f'[node-check] OK={ok_count}  FAIL={fail_count}  '
            f'(total {ok_count + fail_count})')
        log('=' * 64)

    def _on_launch_output(self, key: str):
        proc = self._procs.get(key)
        if proc is None:
            return
        try:
            data = bytes(proc.readAllStandardOutput()).decode('utf-8', errors='replace')
        except Exception:
            data = ''
        if not data:
            return
        for line in data.rstrip('\n').splitlines():
            self._append_log(f'[{key}] {line}')

    def _on_launch_finished(self, key: str):
        self._set_status_dot(key, False)
        self._append_log(f'[{key}] exited')

    def _set_status_dot(self, key: str, running: bool):
        lbl = getattr(self, f'lbl_status_{key}', None)
        if lbl is None:
            return
        if running:
            lbl.setStyleSheet('QLabel { background:#3fa83f; border-radius:8px; }')
        else:
            lbl.setStyleSheet('QLabel { background:#555; border-radius:8px; }')

    def _append_log(self, line: str):
        self.txt_log.appendPlainText(line)

    def closeEvent(self, event):
        # Make sure no orphaned motor processes are left running.
        self._stop_all_launches()
        super().closeEvent(event)

    # --------- periodic refresh
    def _refresh(self):
        snap = self.ros_node.snapshot()

        # cmd_vel readouts
        self.lbl_linear_value.setText(f"{snap['linear']:+0.2f}")
        self.lbl_angular_value.setText(f"{snap['angular']:+0.2f}")

        # /joy Hz
        self.lbl_joy_hz.setText(f"/joy: {snap['joy_hz']:.0f} Hz")

        # Tab 1 joystick live view — axes + 현재 눌린 button 명시
        if hasattr(self, 'lbl_joy_live'):
            self._update_joy_live()

        # connection status (= joy alive); dead-man = real button state from teleop
        alive = snap['joy_alive']
        if alive:
            self.lbl_conn_status.setText(self._conn_text(True))
        else:
            self.lbl_conn_status.setText(self._conn_text(False))

        # Bridge (ESP32 USB-CDC) connection badge — based on /joint_states freshness.
        bridge_state = snap['bridge_state']
        bridge_lbl = getattr(self, 'lbl_bridge_status', None)
        if bridge_lbl is not None:
            self._set_bridge_badge(bridge_lbl, bridge_state)
        self._set_deadman(bool(snap['deadman_active']))
        if self._prev_joy_alive != alive:
            log = self.ros_node.get_logger()
            if alive:
                log.info(f"/joy stream ALIVE  ({snap['joy_hz']:.0f} Hz)")
            else:
                log.warn('/joy stream IDLE (no messages within 1s)')
            self._prev_joy_alive = alive

        # E-stop banner mirrors teleop's published state (single source of truth)
        latched = snap['estop_latched']
        if latched != self._estop_latched:
            self._estop_latched = latched
            self._set_estop_banner(latched)

        # Firmware RPM limit display — refresh labels only when a new #I lands,
        # to avoid clobbering the styling on every 50ms tick.
        if hasattr(self, 'lbl_fw_source'):
            seq = snap['motor_limit_seq']
            if seq != self._last_fw_seq_seen:
                self._last_fw_seq_seen = seq
                self._update_fw_labels(snap['motor_limit_info'])
                if snap['motor_limit_info'] is not None:
                    rl, rr, nl, nr, src = snap['motor_limit_info']
                    self._settings_log(
                        f'[fw-info] ram=({rl},{rr})  nvs=({nl},{nr})  '
                        f'source={self._FW_SOURCE_LABELS.get(int(src), src)}')

    # --------- Tab 1 joystick live display
    # Microntek USB Joystick (PSX clone) axis + button name map.
    # axes (6):    [LStickX, LStickY, RStickX, RStickY, DPadX, DPadY]
    # buttons (12): [X, ○, □, △, L1, R1, L2, R2, Sel, Start, LCk, RCk]
    _JOY_AXIS_NAMES = ['LSx', 'LSy', 'RSx', 'RSy', 'DPadX', 'DPadY']
    _JOY_BTN_NAMES = ['X', '○', '□', '△', 'L1', 'R1', 'L2', 'R2',
                      'Sel', 'Start', 'LCk', 'RCk']

    def _update_joy_live(self):
        """lbl_joy_live 에 최신 /joy stick axes 한 줄 표시 (50Hz 호출).
        Button (L2/R2 deadman 등) 은 상단 DEAD-MAN indicator 가 표시."""
        j = self.ros_node.latest_joy()
        if j is None:
            self.lbl_joy_live.setText('(no /joy data yet)')
            return
        axes, _buttons = j
        parts = []
        for i, a in enumerate(axes):
            name = self._JOY_AXIS_NAMES[i] if i < len(self._JOY_AXIS_NAMES) else f'a{i}'
            mark = '*' if abs(a) > 0.05 else ' '
            parts.append(f'{mark}{name}: {a:+.2f}')
        self.lbl_joy_live.setText('   '.join(parts))

    # --------- visual helpers
    @staticmethod
    def _conn_text(alive: bool) -> str:
        if alive:
            return '●  /joy active'
        return '●  no /joy stream'

    # --------- Settings tab (velocity limits)
    def _settings_log(self, msg: str):
        if hasattr(self, 'txt_settings_log'):
            self.txt_settings_log.appendPlainText(msg)

    def _settings_target_node(self) -> str:
        if hasattr(self, 'cmb_target_node'):
            return self.cmb_target_node.currentText().strip() or '/microntek_teleop'
        return '/microntek_teleop'

    def _on_settings_refresh(self):
        """Read current max_linear_velocity / max_angular_velocity from the
        selected teleop node and populate the spin boxes (angular shown in deg/s)."""
        node = self._settings_target_node()
        vals = self.ros_node.get_double_params(
            node, ['max_linear_velocity', 'max_angular_velocity'])
        if not vals:
            self._settings_log(f'[refresh] {node} unreachable — '
                               f'is the teleop launch running?')
            self.lbl_linear_current.setText('(current: --)')
            self.lbl_angular_current.setText('(current: --)')
            return
        lin = vals.get('max_linear_velocity')
        ang_rad = vals.get('max_angular_velocity')
        if lin is not None:
            self.lbl_linear_current.setText(f'(current: {lin:.3f} m/s)')
            self.spin_max_linear.setValue(round(lin, 3))
        if ang_rad is not None:
            ang_deg = math.degrees(ang_rad)
            self.lbl_angular_current.setText(
                f'(current: {ang_rad:.4f} rad/s = {ang_deg:.2f} deg/s)')
            self.spin_max_angular_deg.setValue(round(ang_deg, 2))
        self._settings_log(f'[refresh] {node}: '
                           f'lin={lin}, ang={ang_rad} rad/s')

    def _on_settings_apply(self):
        """Send the current spin-box values to the selected teleop node via
        SetParameters. Angular is converted from deg/s (UI) to rad/s (ROS)."""
        node = self._settings_target_node()
        lin = float(self.spin_max_linear.value())
        ang_rad = math.radians(float(self.spin_max_angular_deg.value()))
        ok, msg = self.ros_node.set_double_params(node, {
            'max_linear_velocity': lin,
            'max_angular_velocity': ang_rad,
        })
        if ok:
            self._settings_log(
                f'[apply] {node} ✓  lin={lin:.3f} m/s, '
                f'ang={ang_rad:.4f} rad/s ({self.spin_max_angular_deg.value():.1f} deg/s)')
            # Update the "(current: ...)" caption to reflect what we just wrote.
            self.lbl_linear_current.setText(f'(current: {lin:.3f} m/s)')
            self.lbl_angular_current.setText(
                f'(current: {ang_rad:.4f} rad/s = '
                f'{self.spin_max_angular_deg.value():.2f} deg/s)')
        else:
            self._settings_log(f'[apply] {node} ✗  {msg}')

    def _on_settings_defaults(self):
        """Reset spin boxes to project policy: 0.3 m/s, 15 deg/s. Does NOT
        apply — user still has to press APPLY."""
        self.spin_max_linear.setValue(2.0)
        self.spin_max_angular_deg.setValue(30.0)
        self._settings_log('[reset] spin boxes set to 2.0 m/s, 30.0 deg/s — '
                           'press APPLY to push to teleop node')

    # --------- Firmware speed limits (protocol v3: #L SET / #G GET / #I INFO)
    # Wire format uses motor RPM, but the GUI exposes m/s and deg/s for unit
    # parity with the teleop velocity-limits group. Conversion uses the
    # diff_drive_controller's wheel_radius / wheel_base, queried at
    # sequence-complete time.
    _FW_SOURCE_LABELS = {
        0: 'NVS_LOAD',         # loaded from NVS at boot
        1: 'DEFAULT_USED',     # NVS empty (first boot), defaults seeded
        2: 'RAM_OVERRIDE',     # last SET was persist=0; RAM != NVS
        3: 'SANITIZED',        # NVS had bad values, defaults applied
    }

    def _refresh_wheel_kinematics(self):
        """Query diff_drive_controller for wheel_radius / wheel_base. The
        m/s↔RPM and deg/s↔RPM conversions need these to stay consistent
        with diff_drive_params.yaml — hardcoding would silently drift if the
        yaml changes."""
        vals = self.ros_node.get_double_params(
            '/diff_drive_controller', ['wheel_radius', 'wheel_base'])
        if 'wheel_radius' in vals and 'wheel_base' in vals:
            self._wheel_radius = float(vals['wheel_radius'])
            self._wheel_base = float(vals['wheel_base'])
            if hasattr(self, 'lbl_fw_kine'):
                self.lbl_fw_kine.setText(
                    f'wheel_radius={self._wheel_radius:.3f} m  '
                    f'wheel_base={self._wheel_base:.3f} m')
            self._settings_log(
                f'[fw-kinematics] r={self._wheel_radius:.3f}m '
                f'base={self._wheel_base:.3f}m')
        else:
            self._settings_log(
                '[fw-kinematics] /diff_drive_controller params unreachable — '
                'using fallback (0.085 / 0.330)')

    def _rpm_lin_to_mps(self, rpm: float) -> float:
        # straight motion: both wheels at rpm → robot speed = ω_wheel × r
        return rpm * 2.0 * math.pi * self._wheel_radius / 60.0

    def _mps_to_rpm_lin(self, mps: float) -> float:
        return mps * 60.0 / (2.0 * math.pi * self._wheel_radius)

    def _rpm_rot_to_degs(self, rpm: float) -> float:
        # pure rotation: wheels at ±rpm in opposite directions
        # ω_robot = 2 × wheel_speed / wheel_base = 2 × ω_wheel × r / wheel_base
        # deg/s = ω_robot × 180/π = 12 × rpm × r / base
        if self._wheel_base <= 0:
            return 0.0
        return 12.0 * rpm * self._wheel_radius / self._wheel_base

    def _degs_to_rpm_rot(self, degs: float) -> float:
        if self._wheel_radius <= 0:
            return 0.0
        return degs * self._wheel_base / (12.0 * self._wheel_radius)

    def _on_fw_apply(self, persist: bool):
        """Send #L (LIMIT_SET) to the bridge. Spinbox values are in m/s and
        deg/s — converted to motor RPM here using cached wheel kinematics."""
        mps = float(self.spin_fw_max_lin.value())
        degs = float(self.spin_fw_max_rot.value())
        rpm_lin = max(1, min(3000, int(round(self._mps_to_rpm_lin(mps)))))
        rpm_rot = max(1, min(3000, int(round(self._degs_to_rpm_rot(degs)))))
        self.ros_node.publish_motor_limit_set(rpm_lin, rpm_rot, persist)
        mode = 'NVS-persist' if persist else 'RAM-only'
        self._settings_log(
            f'[fw-apply {mode}] {mps:.2f} m/s → {rpm_lin} RPM, '
            f'{degs:.1f}°/s → {rpm_rot} RPM  '
            f'(persist={1 if persist else 0}) — sent #L, wait for #I')

    def _on_fw_refresh(self):
        """Send #G (LIMIT_GET) AND re-fetch wheel kinematics. The kinematics
        also refreshes here so a manual REFRESH always catches up — the
        sequence-complete auto-fetch can race the param service coming up.
        Updates RAM/NVS labels only — does NOT touch spinbox (use LOAD FROM
        NVS for that)."""
        self._refresh_wheel_kinematics()
        self.ros_node.publish_motor_limit_get()
        self._settings_log('[fw-refresh] sent #G — waiting for #I')

    def _on_fw_load_nvs(self):
        """Read EEPROM/NVS-saved values and populate spinboxes with them.
        Sends #G; the next incoming #I is consumed to set spinbox values from
        the nvs_lin/nvs_rot fields (the persisted state that survives reboot).
        User can then modify and APPLY (RAM-only) for transient test, or
        APPLY & SAVE (NVS) for permanent change."""
        self._load_nvs_pending = True
        self.ros_node.publish_motor_limit_get()
        self._settings_log(
            '[fw-load-nvs] sent #G — waiting for #I to fill spinbox from NVS')

    def _update_fw_labels(self, info):
        """Update RAM/NVS/source labels from a motor_limit_info tuple
        (ram_lin, ram_rot, nvs_lin, nvs_rot, source). RAM/NVS RPMs converted
        to m/s and deg/s for display; raw RPM shown in parentheses for
        firmware-level cross-reference. info=None → '--'.

        Spinbox population is gated by an explicit user action (LOAD FROM NVS
        button) — NEVER auto-overrides the user's typing on every #I."""
        if info is None:
            self.lbl_fw_lin_status.setText('RAM: --   NVS: --')
            self.lbl_fw_rot_status.setText('RAM: --   NVS: --')
            self.lbl_fw_source.setText('--')
            return
        ram_lin, ram_rot, nvs_lin, nvs_rot, source = info
        lin_mismatch = ram_lin != nvs_lin
        rot_mismatch = ram_rot != nvs_rot
        lin_color = '#ffe070' if lin_mismatch else '#ddd'
        rot_color = '#ffe070' if rot_mismatch else '#ddd'
        ram_lin_mps = self._rpm_lin_to_mps(ram_lin)
        nvs_lin_mps = self._rpm_lin_to_mps(nvs_lin)
        ram_rot_degs = self._rpm_rot_to_degs(ram_rot)
        nvs_rot_degs = self._rpm_rot_to_degs(nvs_rot)
        self.lbl_fw_lin_status.setText(
            f'RAM: {ram_lin_mps:.2f} m/s ({ram_lin} RPM)   '
            f'NVS: {nvs_lin_mps:.2f} m/s ({nvs_lin} RPM)')
        self.lbl_fw_lin_status.setStyleSheet(
            f'QLabel {{ color:{lin_color}; font-family:monospace; }}')
        self.lbl_fw_rot_status.setText(
            f'RAM: {ram_rot_degs:.1f}°/s ({ram_rot} RPM)   '
            f'NVS: {nvs_rot_degs:.1f}°/s ({nvs_rot} RPM)')
        self.lbl_fw_rot_status.setStyleSheet(
            f'QLabel {{ color:{rot_color}; font-family:monospace; }}')
        # If user clicked LOAD FROM NVS, the next incoming #I is consumed to
        # populate the spinboxes with the NVS-saved values (the EEPROM-stored
        # state — what survives reboot).
        if getattr(self, '_load_nvs_pending', False):
            self.spin_fw_max_lin.blockSignals(True)
            self.spin_fw_max_lin.setValue(round(nvs_lin_mps, 2))
            self.spin_fw_max_lin.blockSignals(False)
            self.spin_fw_max_rot.blockSignals(True)
            self.spin_fw_max_rot.setValue(round(nvs_rot_degs, 1))
            self.spin_fw_max_rot.blockSignals(False)
            self._load_nvs_pending = False
            self._settings_log(
                f'[fw-load-nvs] spinbox ← NVS  '
                f'lin={nvs_lin_mps:.2f} m/s ({nvs_lin} RPM)  '
                f'rot={nvs_rot_degs:.1f}°/s ({nvs_rot} RPM)')
        src_label = self._FW_SOURCE_LABELS.get(int(source), f'?{source}')
        src_color = {
            'NVS_LOAD': '#7fff7f',       # green = persisted state in use
            'DEFAULT_USED': '#7fcfff',   # cyan = first boot
            'RAM_OVERRIDE': '#ffe070',   # yellow = transient override
            'SANITIZED': '#ff7f7f',      # red = NVS was corrupt
        }.get(src_label, '#ddd')
        self.lbl_fw_source.setText(f'{src_label}  ({source})')
        self.lbl_fw_source.setStyleSheet(
            f'QLabel {{ color:{src_color}; font-family:monospace; font-weight:bold; }}')

    def _on_motor_enable_toggled(self, checked: bool):
        """Toggle handler for the MOTOR ENABLE/DISABLE button on Tab 1.

        On ENABLE: publish /clear_error first (release any latched fault),
        then /motor_enable=true. On DISABLE: publish /motor_enable=false.
        """
        self.ros_node.publish_motor_enable(checked)
        btn = self.btn_motor_enable
        if checked:
            btn.setText('MOTORS ENABLED')
            self._append_log('[motor_enable] /clear_error + /motor_enable=true sent')
        else:
            btn.setText('MOTORS DISABLED')
            self._append_log('[motor_enable] /motor_enable=false sent')

    def _on_freewheel_clicked(self):
        """Send /freewheel → firmware: motor OFF + brake released. Sync the
        MOTORS ENABLED toggle to off-state so the GUI doesn't show conflicting
        state (since firmware is now in READY/freewheel, not ENABLED)."""
        self.ros_node.publish_freewheel()
        if hasattr(self, 'btn_motor_enable') and self.btn_motor_enable.isChecked():
            # Uncheck without re-publishing /motor_enable=false (firmware is
            # already at READY after #F). blockSignals avoids the toggled signal.
            self.btn_motor_enable.blockSignals(True)
            self.btn_motor_enable.setChecked(False)
            self.btn_motor_enable.setText('MOTORS DISABLED')
            self.btn_motor_enable.blockSignals(False)
        self._append_log('[freewheel] /freewheel sent — motor OFF + brake released')

    # --------- Pipeline Health (Tab 4)
    # Each chain link in CHAIN_LINKS is one table row. The probe_fn returns
    # (state_str, value_str, detail_str) where state_str ∈ {'ok','warn','down','idle'}.
    # See docs/sw-architecture/pipeline_chain_failure_modes_2026-05-08.md for
    # the canonical link catalog (L1..L10 plus state probes).
    _HEALTH_COLUMNS = ['', 'Link', 'Value', 'Detail']

    def _init_health_table(self):
        from PyQt5.QtWidgets import QTableWidgetItem, QHeaderView
        self._QTableWidgetItem = QTableWidgetItem
        rows = self._chain_link_definitions()
        self.tbl_health.setColumnCount(len(self._HEALTH_COLUMNS))
        self.tbl_health.setHorizontalHeaderLabels(self._HEALTH_COLUMNS)
        self.tbl_health.setRowCount(len(rows))
        # Pre-populate row 0 (status dot) and row 1 (link name) — these don't change.
        for r, (_id, name, _probe) in enumerate(rows):
            dot = QTableWidgetItem('●')
            dot.setTextAlignment(QtCore.Qt.AlignCenter)
            self.tbl_health.setItem(r, 0, dot)
            self.tbl_health.setItem(r, 1, QTableWidgetItem(name))
            self.tbl_health.setItem(r, 2, QTableWidgetItem('--'))
            self.tbl_health.setItem(r, 3, QTableWidgetItem(''))
        # Column sizing
        h = self.tbl_health.horizontalHeader()
        h.setSectionResizeMode(0, QHeaderView.Fixed)
        self.tbl_health.setColumnWidth(0, 32)
        h.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        h.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        h.setSectionResizeMode(3, QHeaderView.Stretch)
        self.tbl_health.verticalHeader().setVisible(False)
        # 1Hz refresh timer
        self._health_timer = QtCore.QTimer(self)
        self._health_timer.timeout.connect(self._refresh_health)
        self._health_timer.start(1000)

    def _chain_link_definitions(self):
        """Return ordered list of (link_id, display_name, probe_fn). probe_fn
        takes a snapshot dict and returns (state, value_str, detail)."""
        return [
            ('L1', 'L1: joystick (HW)',           self._probe_joystick_hw),
            ('L2', 'L2: joy_linux',               self._probe_joy_linux_node),
            ('L3', 'L3: /joy',                    self._probe_joy_topic),
            ('L4', 'L4: microntek_teleop',        self._probe_teleop_node),
            ('L5', 'L5: /cmd_vel (Stage 1 cap)',  self._probe_cmd_vel),
            ('L6', 'L6: diff_drive_controller',   self._probe_diff_drive_node),
            ('L7', 'L7: motor_rpm',               self._probe_motor_rpm),
            ('L8', 'L8: bridge ↔ ESP32',          self._probe_bridge),
            ('L9', 'L9: ESP32 firmware',          self._probe_firmware),
            ('L10', 'L10: motors',                self._probe_motors),
        ]

    @staticmethod
    def _classify_age(age, ok=1.0, warn=5.0):
        """age (seconds since last data) → 'ok' / 'warn' / 'down' / 'idle'.
        None age → 'idle' (no data ever received)."""
        if age is None:
            return 'idle'
        if age <= ok:
            return 'ok'
        if age <= warn:
            return 'warn'
        return 'down'

    # ---- per-link probes ----
    def _probe_joystick_hw(self, snap):
        # L1 — kernel device file presence (/dev/input/js0 or /dev/input/joystick_wired)
        present = (os.path.exists('/dev/input/js0') or
                   os.path.exists('/dev/input/joystick_wired'))
        if not present:
            return 'down', 'no /dev/input/js*', 'plug joystick or check ANALOG mode'
        # If /joy is flowing but axes are all zero → suspect ANALOG mode OFF
        joy_hz, joy_age = snap['/joy']
        return 'ok', '/dev/input/js0 present', 'L1.A: ANALOG OFF if /joy axes all zero'

    def _probe_joy_linux_node(self, _snap):
        # L2 — node alive + dup detection
        try:
            nodes = self.ros_node.get_node_names()
        except Exception:
            return 'down', 'rclpy err', ''
        # joy_linux is unique-named; multiple instances would cause dup pubs but
        # appear as a single name in get_node_names(). Cross-check pub count.
        alive = ('joy_linux' in nodes or '/joy_linux' in nodes)
        if not alive:
            return 'down', 'node missing', 'restart microntek_teleop launch'
        try:
            pubs = self.ros_node.get_publishers_info_by_topic('/joy')
            pub_count = len(pubs)
        except Exception:
            pub_count = 1
        if pub_count > 1:
            return 'warn', f'pubs={pub_count}', 'L2.A dup-dev hijack — kill duplicates'
        return 'ok', f'pubs={pub_count}', ''

    def _probe_joy_topic(self, snap):
        hz, age = snap['/joy']
        state = self._classify_age(age)
        return state, f'{hz:.1f} Hz', f'last {age*1000:.0f} ms ago' if age is not None else 'no data'

    def _probe_teleop_node(self, snap):
        try:
            nodes = self.ros_node.get_node_names()
        except Exception:
            return 'down', 'rclpy err', ''
        alive = ('microntek_teleop' in nodes or '/microntek_teleop' in nodes
                 or 'pro_controller_teleop' in nodes
                 or '/pro_controller_teleop' in nodes)
        if not alive:
            return 'down', 'node missing', 'L4 — teleop not running'
        # Deadman + estop state
        s = self.ros_node.snapshot()
        flags = []
        if s['estop_latched']:
            flags.append('ESTOP_LATCHED')
        if not s['deadman_active']:
            flags.append('deadman_off')
        msg = ', '.join(flags) if flags else 'ready'
        # If teleop is up but cmd_vel idle and deadman off → expected
        return ('ok' if not s['estop_latched'] else 'warn'), 'alive', msg

    def _probe_cmd_vel(self, snap):
        hz, age = snap['/cmd_vel']
        state = self._classify_age(age)
        s = self.ros_node.snapshot()
        return state, f'{hz:.1f} Hz', \
            f'lin={s["linear"]:+0.2f}, ang={s["angular"]:+0.3f}'

    def _probe_diff_drive_node(self, _snap):
        try:
            nodes = self.ros_node.get_node_names()
        except Exception:
            return 'down', 'rclpy err', ''
        alive = ('diff_drive_controller' in nodes or '/diff_drive_controller' in nodes)
        return ('ok', 'alive', '') if alive else ('down', 'node missing', 'L6 — diff_drive not running')

    def _probe_motor_rpm(self, snap):
        l_hz, l_age = snap['/fito_amr/left_motor_rpm']
        r_hz, r_age = snap['/fito_amr/right_motor_rpm']
        # Worst of the two
        state = self._classify_age(min(filter(lambda a: a is not None,
                                              [l_age, r_age]), default=None))
        return state, f'L:{l_hz:.0f}/R:{r_hz:.0f} Hz', 'diff_drive → bridge'

    def _probe_bridge(self, snap):
        try:
            nodes = self.ros_node.get_node_names()
        except Exception:
            return 'down', 'rclpy err', ''
        alive = ('fito_amr_serial_bridge' in nodes
                 or '/fito_amr_serial_bridge' in nodes)
        if not alive:
            return 'down', 'node missing', 'L8.C — bridge crashed'
        # Bridge alive — RX freshness via /joint_states (proxy)
        _hz, age = snap['/joint_states']
        s = self.ros_node.snapshot()
        bridge_state = s['bridge_state']
        if bridge_state == 'connected':
            return 'ok', 'connected', f'/joint_states {age*1000:.0f} ms'
        if bridge_state == 'reconnecting':
            return 'warn', 'reconnecting', 'transient stall'
        return 'down', 'DOWN', 'L8.A/B — USB drop, see issues_and_fixes/2026-05-08'

    def _probe_firmware(self, snap):
        # L9 — firmware health proxied via /joint_states freshness (bridge needs
        # to be up too). If bridge is down, firmware health unknown.
        s = self.ros_node.snapshot()
        if s['bridge_state'] == 'down':
            return 'idle', 'bridge down', 'firmware reachability unknown'
        _hz, age = snap['/joint_states']
        if age is None or age > 2.0:
            return 'down', 'no #J/#H', 'L9.A — TWAI lockup or fw crash'
        return 'ok', f'#J {1.0/max(age,0.001):.0f} Hz', 'st:running (see bridge log for fw st/alarm)'

    def _probe_motors(self, snap):
        # L10 — motors. Indirect: motor_alive heartbeat + #J flowing.
        _hz, age = snap['/motor_alive']
        s = self.ros_node.snapshot()
        if s['bridge_state'] == 'down':
            return 'idle', '?', 'bridge down'
        state = self._classify_age(age, ok=2.0, warn=10.0)
        if age is None:
            return 'idle', 'no /motor_alive', 'motor_health_monitor not up?'
        return state, f'last {age:.1f}s', 'motor_health_monitor heartbeat'

    def _refresh_health(self):
        """1 Hz update of the Pipeline Health table + summary banner."""
        if not hasattr(self, 'tbl_health') or not hasattr(self, '_QTableWidgetItem'):
            return
        snap = self.ros_node.pipeline_snapshot()
        rows = self._chain_link_definitions()
        ok = warn = down = idle = 0
        color_map = {
            'ok':   '#7fff7f',
            'warn': '#ffe070',
            'down': '#ff7f7f',
            'idle': '#888888',
        }
        for r, (_id, _name, probe_fn) in enumerate(rows):
            try:
                state, value, detail = probe_fn(snap)
            except Exception as exc:
                state, value, detail = 'down', 'probe err', str(exc)[:60]
            ok += int(state == 'ok')
            warn += int(state == 'warn')
            down += int(state == 'down')
            idle += int(state == 'idle')
            dot = self.tbl_health.item(r, 0)
            dot.setForeground(QtCore.Qt.GlobalColor.gray)
            from PyQt5.QtGui import QColor
            dot.setForeground(QColor(color_map[state]))
            self.tbl_health.item(r, 2).setText(value)
            self.tbl_health.item(r, 3).setText(detail)
        # Summary banner
        total = ok + warn + down + idle
        if down > 0:
            banner = f'✗ {down} link(s) DOWN  ·  ⚠ {warn}  ·  ✓ {ok}  ·  idle {idle}'
            color = '#ff7f7f'; bg = '#3a1e1e'; border = '#c04040'
        elif warn > 0:
            banner = f'⚠ {warn} link(s) degraded  ·  ✓ {ok}  ·  idle {idle}'
            color = '#ffe070'; bg = '#3a3a1e'; border = '#c0a040'
        elif idle == total:
            banner = f'idle — chain not started yet ({idle}/{total})'
            color = '#aaa'; bg = '#2a2a2a'; border = '#555'
        else:
            banner = f'✓ all {ok} active link(s) healthy  ·  idle {idle}'
            color = '#7fff7f'; bg = '#1e3a1e'; border = '#3fa83f'
        if hasattr(self, 'lbl_health_summary'):
            self.lbl_health_summary.setText(banner)
            self.lbl_health_summary.setStyleSheet(
                f'QLabel {{ background:{bg}; color:{color}; '
                f'border:1px solid {border}; border-radius:6px; '
                'padding:8px; font-size:13pt; font-weight:bold; }}')

    @staticmethod
    def _set_bridge_badge(label, state: str):
        """Update bridge connection badge based on /joint_states freshness.

        state in {'connected', 'reconnecting', 'down'}.
        """
        if state == 'connected':
            label.setText('●  bridge: connected')
            label.setStyleSheet(
                'QLabel { background:#1e3a1e; color:#7fff7f; '
                'border:1px solid #3fa83f; border-radius:4px; padding:6px; font-weight:bold; }')
        elif state == 'reconnecting':
            label.setText('●  bridge: reconnecting...')
            label.setStyleSheet(
                'QLabel { background:#3a3a1e; color:#ffe070; '
                'border:1px solid #c0a040; border-radius:4px; padding:6px; font-weight:bold; }')
        else:  # 'down'
            label.setText('●  bridge: DOWN')
            label.setStyleSheet(
                'QLabel { background:#3a1e1e; color:#ff7f7f; '
                'border:1px solid #c04040; border-radius:4px; padding:6px; font-weight:bold; }')

    def _set_deadman(self, on: bool):
        if on:
            self.lbl_deadman.setStyleSheet(
                'QLabel { background:#1e3a1e; color:#7fff7f; '
                'border:1px solid #3fa83f; border-radius:4px; font-weight:bold; }')
        else:
            self.lbl_deadman.setStyleSheet(
                'QLabel { background:#333; color:#888; '
                'border:1px solid #555; border-radius:4px; font-weight:bold; }')

    def _set_estop_banner(self, latched: bool):
        if latched:
            self.lbl_estop_banner.setText('LATCHED')
            self.lbl_estop_banner.setStyleSheet(
                'QLabel { background:#a02020; color:white; '
                'border:3px solid #ff4040; border-radius:8px; '
                'font-size:32pt; font-weight:bold; }')
        else:
            self.lbl_estop_banner.setText('READY')
            self.lbl_estop_banner.setStyleSheet(
                'QLabel { background:#1f3a1f; color:#7fff7f; '
                'border:3px solid #3fa83f; border-radius:8px; '
                'font-size:32pt; font-weight:bold; }')


# --------------------------------------------------------------------- main
def main(args=None):
    rclpy.init(args=args)
    ros_node = TeleopGuiNode()

    pkg_share = get_package_share_directory('joystick_gui')
    ui_path = os.path.join(pkg_share, 'ui', 'teleop_panel.ui')

    app = QtWidgets.QApplication(sys.argv)
    panel = TeleopPanel(ros_node, ui_path)
    panel.show()

    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
