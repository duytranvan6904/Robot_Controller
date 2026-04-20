#!/usr/bin/env python3
"""
cartesian_streamer_hc10dtp.py
────────────────────────────
Nhận tọa độ Cartesian (PoseStamped) từ AI/Camera node,
giải IK và stream joint angles xuống MotoROS2
qua chế độ Point Queue Mode — phiên bản cho HC10DTP.

Quy trình khởi động (2 terminal):
  Terminal 1: ros2 launch hc10dtp_moveit_config hc10dtp_start.launch.py
              (khởi động: move_group, RViz, robot_state_publisher, restamp_joint_states)
  Terminal 2: python3 cartesian_streamer_hc10dtp.py --demo line

Lưu ý kiến trúc:
  MotoROS2 driver chạy trực tiếp trên YRC1000/YRC1000micro
  và tự expose /yaskawa/* services.

AI node gửi lệnh qua:
  /cartesian_streamer/target_pose  → geometry_msgs/PoseStamped
  hoặc
  /cartesian_streamer/target_xyz   → Float64MultiArray [x, y, z]
                                     (orientation giữ nguyên từ vị trí hiện tại)

Test tích hợp:
  python3 cartesian_streamer_hc10dtp.py --demo circle
  python3 cartesian_streamer_hc10dtp.py --demo line
  python3 cartesian_streamer_hc10dtp.py --demo lissajous
"""

import math
import threading
import argparse
import time as _time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import PositionIKRequest, RobotState
from motoros2_interfaces.srv import StartPointQueueMode, QueueTrajPoint
from std_srvs.srv import Trigger

# ── Hằng số ────────────────────────────────────────────────────────
JOINT_NAMES = [
    'joint_1_s', 'joint_2_l', 'joint_3_u',
    'joint_4_r', 'joint_5_b', 'joint_6_t'
]
GROUP_NAME   = 'hc10dtp_arm'
EE_LINK      = 'tool0'
BASE_FRAME   = 'base_link'

# Workspace an toàn (mét) — HC10DTP có tầm với 1.2m, điều chỉnh theo môi trường
WS_X = (-1.4,  1.4)
WS_Y = (-1.4,  1.4)
WS_Z = ( 0.05, 1.5)

# ── Tunable stream parameters ──────────────────────────────────────
# Tần suất stream tick (Hz). 20Hz là giá trị ổn định cho MotoROS2
# HC10DTP cobot: 15Hz cho margin an toàn với collaborative mode.
DEFAULT_STREAM_HZ = 15

# Queue point duration: khoảng cách thời gian giữa mỗi điểm.
# Phải ≥ STREAM_PERIOD_SEC. 0.05s = 20Hz motion rate — cho phép
# robot đủ thời gian xử lý trước khi điểm tiếp theo đến.
QUEUE_DT_SEC = 0.067  # ~15Hz motion rate cho HC10DTP
QUEUE_RETRY_BACKOFF_SEC = 0.015

# IK timeout
IK_TIMEOUT_SEC = 0.2

# Smooth: tỉ lệ tiến về target mỗi tick (0.0–1.0)
SMOOTH_ALPHA = 0.4       # Thấp hơn GP4 cho chuyển động mượt hơn (cobot)

# An toàn: bước nhảy joint tối đa cho phép mỗi điểm (rad)
MAX_JOINT_DELTA = 0.3    # Thấp hơn GP4 cho collaborative safety

# Debug watchdog: nếu queue point được accept nhưng joint gần như đứng yên
NO_MOTION_WARN_SEC = 5.0
NO_MOTION_EPS_RAD = 1e-3
NO_MOTION_MIN_ACCEPTED_POINTS = 10
QUEUE_PREBUFFER_POINTS = 3
STREAM_STATE_IDLE = 'idle'
STREAM_STATE_SEEDING = 'seeding'
STREAM_STATE_PREBUFFERING = 'prebuffering'
STREAM_STATE_STREAMING = 'streaming'


class CartesianStreamer(Node):

    def __init__(
        self,
        stream_hz: float = DEFAULT_STREAM_HZ,
        queue_dt_sec: float = QUEUE_DT_SEC,
        prebuffer_points: int = QUEUE_PREBUFFER_POINTS,
        retry_backoff_sec: float = QUEUE_RETRY_BACKOFF_SEC,
    ):
        super().__init__('cartesian_streamer')
        self._stream_hz = stream_hz
        self._stream_period_sec = 1.0 / stream_hz

        self._cb = ReentrantCallbackGroup()

        # ── State ────────────────────────────────────────────────
        self._current_joints: list[float] = [0.0] * 6
        self._got_joints = False
        self._queue_mode_active = False
        self._stream_state = STREAM_STATE_IDLE
        self._queue_call_inflight = False
        self._accepted_points = 0
        self._queue_debug_log_count = 0
        self._active_queue_service_name = ''
        self._pending_point_to_resend: JointTrajectoryPoint | None = None
        self._last_queued_joints: list[float] = [0.0] * 6
        self._queue_dt_sec = max(queue_dt_sec, 0.01)
        self._prebuffer_target = max(prebuffer_points, 1)
        self._retry_backoff_sec = max(retry_backoff_sec, 0.0)
        self._accepted_since_seed = 0
        self._seed_request_sent = False
        # time_from_start phải tích lũy (cumulative) từ seed point.
        # MotoROS2 dùng cumulative timestamp.
        self._cumulative_time_ns: int = 0
        # Lock bảo vệ _queue_call_inflight khỏi race condition
        self._send_lock = threading.Lock()
        self._next_send_not_before_ns = self.get_clock().now().nanoseconds
        self._last_motion_time = self.get_clock().now()
        self._last_warn_time = self.get_clock().now()
        self._tick_count = 0
        self._queue_sent_count = 0
        self._rate_window_start = self.get_clock().now()
        self._window_ack_count = 0
        self._window_busy_count = 0
        self._window_retry_count = 0
        self._window_reject_count = 0
        self._window_max_joint_delta = 0.0
        self._last_ack_time = None
        self._window_ack_interval_sum = 0.0
        self._window_ack_interval_count = 0
        # Số hold-points liên tiếp đã gửi (không tính vào cumulative time)
        self._hold_point_count = 0

        # Target Cartesian pose (được smooth từng bước)
        self._target_pose: Pose | None = None
        self._last_target_update_ns = 0
        # Pose đang thực sự gửi (smooth intermediate)
        self._current_ee_pose: Pose | None = None

        # IK failure counter — để phát hiện stuck
        self._ik_fail_count = 0
        self._last_ok_joints: list[float] = [0.0] * 6
        self._prev_joint_snapshot: list[float] = [0.0] * 6

        # ── Subscribers ──────────────────────────────────────────
        self._js_sub = self.create_subscription(
            JointState, '/yaskawa/joint_states',
            self._on_joint_state, 10, callback_group=self._cb)

        self._pose_sub = self.create_subscription(
            PoseStamped, '/cartesian_streamer/target_pose',
            self._on_target_pose, 10, callback_group=self._cb)

        # Convenience: chỉ gửi XYZ, orientation giữ nguyên
        self._xyz_sub = self.create_subscription(
            Float64MultiArray, '/cartesian_streamer/target_xyz',
            self._on_target_xyz, 10, callback_group=self._cb)

        # Publisher: vị trí EE hiện tại (để AI biết feedback)
        self._ee_pub = self.create_publisher(
            PoseStamped, '/cartesian_streamer/current_pose', 10)

        # ── Service clients ──────────────────────────────────────
        self._ik_cli = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self._cb)

        self._start_queue_cli = self.create_client(
            StartPointQueueMode, '/yaskawa/start_point_queue_mode',
            callback_group=self._cb)
        self._queue_point_cli = self.create_client(
            QueueTrajPoint, '/yaskawa/queue_traj_point',
            callback_group=self._cb)
        self._queue_point_cli_alt = self.create_client(
            QueueTrajPoint, '/yaskawa/queue_point',
            callback_group=self._cb)
        self._queue_point_cli_alt2 = self.create_client(
            QueueTrajPoint, '/queue_point',
            callback_group=self._cb)

        self._stop_traj_cli = self.create_client(
            Trigger, '/yaskawa/stop_traj_mode',
            callback_group=self._cb)

        self._fk_cli = self.create_client(
            GetPositionFK, '/compute_fk', callback_group=self._cb)

        self._reset_error_cli = self.create_client(
            Trigger, '/yaskawa/reset_error', callback_group=self._cb)

        self._servo_on_cli = self.create_client(
            Trigger, '/yaskawa/servo_on', callback_group=self._cb)

        # ── Timers ───────────────────────────────────────────────
        self._stream_timer = self.create_timer(
            self._stream_period_sec, self._stream_tick, callback_group=self._cb)

        self._startup_timer = self.create_timer(
            0.5, self._startup, callback_group=self._cb)

        self.get_logger().info(
            'CartesianStreamer khởi động.\n'
            f'  Stream rate:       {self._stream_hz:.0f} Hz (period={self._stream_period_sec*1000:.1f}ms)\n'
            f'  Queue dt:          {self._queue_dt_sec:.3f} s\n'
            f'  Prebuffer points:  {self._prebuffer_target}\n'
            f'  Retry backoff:     {self._retry_backoff_sec*1000:.1f} ms\n'
            '  Gửi PoseStamped lên: /cartesian_streamer/target_pose\n'
            '  Gửi XYZ lên:        /cartesian_streamer/target_xyz\n'
            '  Nhận EE pose tại:   /cartesian_streamer/current_pose'
        )

    # ═══════════════════════════════════════════════════════════════
    # JOINT STATE CALLBACK
    # ═══════════════════════════════════════════════════════════════

    def _on_joint_state(self, msg: JointState):
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                self._current_joints[i] = msg.position[msg.name.index(name)]
        if not self._got_joints:
            self._got_joints = True
            self._last_ok_joints = list(self._current_joints)
            self._prev_joint_snapshot = list(self._current_joints)
            self._last_queued_joints = list(self._current_joints)
            self.get_logger().info(
                'Nhận joint_states: '
                + str([f'{v:.3f}' for v in self._current_joints])
            )
            return

        max_joint_change = max(
            abs(a - b) for a, b in zip(self._current_joints, self._prev_joint_snapshot)
        )
        if max_joint_change > NO_MOTION_EPS_RAD:
            self._last_motion_time = self.get_clock().now()
        self._prev_joint_snapshot = list(self._current_joints)

    # ═══════════════════════════════════════════════════════════════
    # TARGET CALLBACKS
    # ═══════════════════════════════════════════════════════════════

    def _on_target_pose(self, msg: PoseStamped):
        """Nhận PoseStamped đầy đủ (position + orientation)."""
        if not self._check_workspace(msg.pose.position):
            return
        self._target_pose = msg.pose
        self._last_target_update_ns = self.get_clock().now().nanoseconds

    def _on_target_xyz(self, msg: Float64MultiArray):
        """
        Nhận chỉ XYZ [x, y, z].
        Orientation được giữ nguyên từ vị trí hiện tại hoặc default.
        """
        if len(msg.data) < 3:
            return
        pos = Point(x=msg.data[0], y=msg.data[1], z=msg.data[2])
        if not self._check_workspace(pos):
            return

        pose = Pose()
        pose.position = pos
        # Giữ orientation hiện tại nếu có, không thì dùng default (EE hướng xuống)
        if self._current_ee_pose is not None:
            pose.orientation = self._current_ee_pose.orientation
        else:
            pose.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
        self._target_pose = pose
        self._last_target_update_ns = self.get_clock().now().nanoseconds

    def _check_workspace(self, pos: Point) -> bool:
        """Kiểm tra điểm nằm trong workspace an toàn."""
        ok = (WS_X[0] <= pos.x <= WS_X[1] and
              WS_Y[0] <= pos.y <= WS_Y[1] and
              WS_Z[0] <= pos.z <= WS_Z[1])
        if not ok:
            self.get_logger().warn(
                f'Ngoài workspace: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})\n'
                f'  X: {WS_X}, Y: {WS_Y}, Z: {WS_Z}',
                throttle_duration_sec=1.0
            )
        return ok

    # ═══════════════════════════════════════════════════════════════
    # STARTUP
    # ═══════════════════════════════════════════════════════════════

    def _startup(self):
        """Khởi động tuần tự các chế độ driver."""
        if not self._got_joints:
            self.get_logger().info('Chờ /yaskawa/joint_states...', throttle_duration_sec=2.0)
            return
        self._startup_timer.cancel()

        # Bước 1: Reset Error
        self.get_logger().info('Đang gọi reset_error...')
        self._call_trigger_chained(self._reset_error_cli, 'reset_error', self._step_2_servo_on)

    def _step_2_servo_on(self):
        self.get_logger().info('Đang gọi servo_on...')
        self._call_trigger_chained(self._servo_on_cli, 'servo_on', self._step_3_stop_traj_mode)

    def _step_3_stop_traj_mode(self):
        self.get_logger().info('Gọi stop_traj_mode để giải phóng mode cũ...')
        self._call_trigger_chained(self._stop_traj_cli, 'stop_traj_mode', self._step_4_start_queue_mode)

    def _step_4_start_queue_mode(self):
        self.get_logger().info('Bật StartPointQueueMode (queue_traj_point streaming)...')
        if not self._start_queue_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Service StartPointQueueMode không khả dụng!')
            return
        fut = self._start_queue_cli.call_async(StartPointQueueMode.Request())
        
        def _done(f):
            res = f.result()
            self.get_logger().info(
                f'StartPointQueueMode response: code={res.result_code.value}, msg="{res.message}"'
            )
            if res.result_code.value == 1:
                self._queue_mode_active = True
                self._stream_state = STREAM_STATE_SEEDING
                self._accepted_since_seed = 0
                self._seed_request_sent = False
                self._pending_point_to_resend = None
                self._cumulative_time_ns = 0          # reset cumulative timer
                self._hold_point_count = 0
                self._next_send_not_before_ns = self.get_clock().now().nanoseconds
                self.get_logger().info('✓ Point Queue Mode active. Sẵn sàng stream.')
            else:
                self.get_logger().error(f'StartPointQueueMode FAILED: {res.message}')
        fut.add_done_callback(_done)

    def _call_trigger_chained(self, client, name, next_step_cb):
        """Helper để gọi service bất kỳ và chuyển sang bước tiếp theo."""
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f'Service {name} không khả dụng, bỏ qua.')
            next_step_cb()
            return

        req = client.srv_type.Request()
        fut = client.call_async(req)
        def _done(f):
            try:
                res = f.result()
                code = getattr(getattr(res, 'result_code', None), 'value', 'N/A')
                msg = getattr(res, 'message', '')
                success = getattr(res, 'success', None)
                if success is None:
                    self.get_logger().info(f'{name}: code={code}, msg="{msg}"')
                else:
                    self.get_logger().info(
                        f'{name}: success={success}, code={code}, msg="{msg}"'
                    )
            except Exception as e:
                self.get_logger().error(f'Error calling {name}: {e}')
            next_step_cb()
        fut.add_done_callback(_done)

    # ═══════════════════════════════════════════════════════════════
    # STREAM LOOP
    # ═══════════════════════════════════════════════════════════════

    def _stream_tick(self):
        try:
            if not self._queue_mode_active or not self._got_joints:
                return

            self._tick_count += 1
            self._log_runtime_rates()
            self._check_no_motion_watchdog()
            now = self.get_clock().now()
            if now.nanoseconds < self._next_send_not_before_ns:
                return

            # ── Bước 0: Khởi tạo ─────────────────────────────────────
            if self._stream_state == STREAM_STATE_SEEDING:
                # Bootstrap initial EE pose via FK
                initial_pose = self._solve_fk_sync(list(self._current_joints))
                if initial_pose:
                    self._current_ee_pose = initial_pose
                    fb = PoseStamped()
                    fb.header.frame_id = BASE_FRAME
                    fb.header.stamp = self.get_clock().now().to_msg()
                    fb.pose = initial_pose
                    self._ee_pub.publish(fb)
                    self.get_logger().info(
                        f'Bootstrap EE pose via FK: '
                        f'({initial_pose.position.x:.4f}, '
                        f'{initial_pose.position.y:.4f}, '
                        f'{initial_pose.position.z:.4f})'
                    )

                # Quy tắc queue mode: điểm đầu tiên phải đúng trạng thái hiện tại, t=0, v=0.
                if not self._seed_request_sent:
                    if self._send_joint_point(list(self._current_joints), force_seed=True):
                        self._seed_request_sent = True
                        self.get_logger().info('Đã gửi seed point (current joints, t=0), chờ ACK...')
                return

            # ── Prebuffering: gửi hold-points (không tăng cumulative time) ──
            if self._stream_state == STREAM_STATE_PREBUFFERING:
                self._send_joint_point(list(self._last_queued_joints), is_hold=True)
                return

            # ── Streaming state ───────────────────────────────────────
            if self._target_pose is None:
                # Nếu chưa có target: gửi hold-point để giữ queue alive,
                # KHÔNG tăng cumulative time.
                self._send_joint_point(list(self._last_queued_joints), is_hold=True)
                return

            # ── Bước 1: Smooth pose (interpolate về target) ──────────
            smoothed = self._smooth_pose(self._target_pose)

            # ── Bước 2: Giải IK ──────────────────────────────────────
            joint_solution = self._solve_ik_sync(smoothed)

            if joint_solution is None:
                # IK thất bại → giữ vị trí cũ (hold-point)
                self._ik_fail_count += 1
                if self._ik_fail_count % 10 == 1:
                    self.get_logger().warn(
                        f'IK thất bại {self._ik_fail_count} lần liên tiếp. '
                        'Robot giữ nguyên vị trí.')
                self._send_joint_point(list(self._last_queued_joints), is_hold=True)
                return

            # ── An toàn: kiểm tra bước nhảy joint ────────────────────
            # So sánh với _last_queued_joints (không phải _current_joints).
            max_delta = max(abs(j - c) for j, c in
                            zip(joint_solution, self._last_queued_joints))
            if max_delta > MAX_JOINT_DELTA:
                self.get_logger().warn(
                    f'IK solution quá xa vị trí hiện tại '
                    f'(max_delta={max_delta:.3f} rad > {MAX_JOINT_DELTA}). '
                    f'Bỏ qua để bảo vệ robot.',
                    throttle_duration_sec=1.0)
                return

            self._ik_fail_count = 0
            self._last_ok_joints = joint_solution
            self._current_ee_pose = smoothed  # cập nhật EE pose

            self.get_logger().info(
                f'IK OK → Δmax={max_delta:.4f} rad, '
                f'joints: [{", ".join(f"{j:.3f}" for j in joint_solution)}]',
                throttle_duration_sec=2.0)

            # ── Bước 3: Gửi xuống robot (motion point) ────────────────
            self._window_max_joint_delta = max(self._window_max_joint_delta, max_delta)
            self._send_joint_point(joint_solution, is_hold=False)

            # ── Bước 4: Publish feedback EE pose ─────────────────────
            fb = PoseStamped()
            fb.header.frame_id = BASE_FRAME
            fb.header.stamp = self.get_clock().now().to_msg()
            fb.pose = smoothed
            self._ee_pub.publish(fb)
        except Exception as e:
            self.get_logger().error(f'_stream_tick exception: {e}')

    # ═══════════════════════════════════════════════════════════════
    # IK SOLVER (thread-safe, không dùng spin_until_future_complete)
    # ═══════════════════════════════════════════════════════════════

    def _solve_ik_sync(self, target_pose: Pose) -> list[float] | None:
        """
        Giải IK đồng bộ cho target_pose.
        Dùng threading.Event để chờ kết quả — an toàn với MultiThreadedExecutor.
        """
        # Build request
        req = GetPositionIK.Request()
        req.ik_request = PositionIKRequest()
        req.ik_request.group_name         = GROUP_NAME
        req.ik_request.ik_link_name       = EE_LINK
        req.ik_request.avoid_collisions   = False  # True sẽ chậm hơn
        req.ik_request.timeout.sec        = 0
        req.ik_request.timeout.nanosec    = int(IK_TIMEOUT_SEC * 1e9)

        # Target pose
        ps = PoseStamped()
        ps.header.frame_id = BASE_FRAME
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.pose            = target_pose
        req.ik_request.pose_stamped = ps

        # Dùng _last_queued_joints làm seed cho IK.
        seed = RobotState()
        seed.joint_state.name     = JOINT_NAMES
        seed.joint_state.position = list(self._last_queued_joints)
        req.ik_request.robot_state = seed

        # Dùng threading.Event để chờ (an toàn với MultiThreadedExecutor)
        event = threading.Event()
        result_holder: list = [None]

        def _done_cb(future):
            result_holder[0] = future
            event.set()

        try:
            ros_future = self._ik_cli.call_async(req)
            ros_future.add_done_callback(_done_cb)

            got_result = event.wait(timeout=IK_TIMEOUT_SEC + 0.01)

            if not got_result or result_holder[0] is None:
                return None

            result = result_holder[0].result()
            # error_code: 1 = SUCCESS
            if result.error_code.val != 1:
                return None

            # Lấy joint positions từ result
            js = result.solution.joint_state
            positions = [0.0] * 6
            for i, name in enumerate(JOINT_NAMES):
                if name in js.name:
                    idx = list(js.name).index(name)
                    positions[i] = js.position[idx]
            return positions

        except Exception as e:
            self.get_logger().error(f'IK exception: {e}', throttle_duration_sec=2.0)
            return None

    def _solve_fk_sync(self, joints: list[float]) -> Pose | None:
        """Giải FK đồng bộ cho list joints."""
        if not self._fk_cli.wait_for_service(timeout_sec=1.0):
            return None

        req = GetPositionFK.Request()
        req.header.frame_id = BASE_FRAME
        req.header.stamp = self.get_clock().now().to_msg()
        req.fk_link_names = [EE_LINK]
        
        seed = RobotState()
        seed.joint_state.name = JOINT_NAMES
        seed.joint_state.position = joints
        req.robot_state = seed

        event = threading.Event()
        result_holder = [None]

        def _done_cb(future):
            result_holder[0] = future
            event.set()

        try:
            ros_future = self._fk_cli.call_async(req)
            ros_future.add_done_callback(_done_cb)
            if not event.wait(timeout=0.5):
                return None
            
            res = result_holder[0].result()
            if res and res.pose_stamped:
                return res.pose_stamped[0].pose
            return None
        except Exception as e:
            self.get_logger().error(f'FK exception: {e}')
            return None

    # ═══════════════════════════════════════════════════════════════
    # POSE SMOOTHING
    # ═══════════════════════════════════════════════════════════════

    def _smooth_pose(self, target: Pose) -> Pose:
        """
        Interpolate từ current_ee_pose về target với hệ số SMOOTH_ALPHA.
        Lần đầu tiên: trả về target ngay lập tức.
        """
        if self._current_ee_pose is None:
            self._current_ee_pose = Pose(
                position=Point(
                    x=target.position.x,
                    y=target.position.y,
                    z=target.position.z,
                ),
                orientation=target.orientation,
            )
            return self._current_ee_pose

        a = SMOOTH_ALPHA
        result = Pose()
        result.position.x = self._lerp(self._current_ee_pose.position.x, target.position.x, a)
        result.position.y = self._lerp(self._current_ee_pose.position.y, target.position.y, a)
        result.position.z = self._lerp(self._current_ee_pose.position.z, target.position.z, a)
        # SLERP orientation
        result.orientation = self._slerp_quat(self._current_ee_pose.orientation, target.orientation, a)
        return result

    @staticmethod
    def _lerp(a: float, b: float, t: float) -> float:
        return a + t * (b - a)

    @staticmethod
    def _slerp_quat(q0: Quaternion, q1: Quaternion, t: float) -> Quaternion:
        """Spherical linear interpolation giữa 2 quaternion."""
        def to_arr(q): return [q.x, q.y, q.z, q.w]
        def dot(a, b): return sum(x*y for x, y in zip(a, b))

        a, b = to_arr(q0), to_arr(q1)
        d = dot(a, b)
        if d < 0:  # chọn shortest path
            b = [-x for x in b]
            d = -d
        d = min(1.0, d)
        if d > 0.9995:  # quá gần → lerp thường
            r = [a[i] + t*(b[i]-a[i]) for i in range(4)]
        else:
            theta0 = math.acos(d)
            theta  = theta0 * t
            sin0, sin1 = math.sin(theta0), math.sin(theta)
            s0 = math.cos(theta) - d * sin1 / sin0
            s1 = sin1 / sin0
            r  = [s0*a[i] + s1*b[i] for i in range(4)]
        norm = math.sqrt(sum(x*x for x in r))
        r = [x/norm for x in r]
        return Quaternion(x=r[0], y=r[1], z=r[2], w=r[3])

    # ═══════════════════════════════════════════════════════════════
    # GỬI ĐIỂM XUỐNG ROBOT (QUEUE_TRAJ_POINT)
    # ═══════════════════════════════════════════════════════════════

    def _send_joint_point(
        self,
        joints: list[float],
        force_seed: bool = False,
        is_hold: bool = False,
    ):
        """
        Gửi một JointTrajectoryPoint xuống MotoROS2 queue.

        Args:
            joints: joint positions [6]
            force_seed: True = seed point (t=0, v=0)
            is_hold: True = hold-point (giữ vị trí, KHÔNG tăng cumulative time).
                     Dùng khi chưa có target hoặc IK thất bại.
        """
        # Thread-safe guard
        if force_seed and self._stream_state != STREAM_STATE_SEEDING:
            self.get_logger().warn(
                f'Bỏ qua seed point vì state hiện tại là {self._stream_state}',
                throttle_duration_sec=1.0
            )
            return False
        with self._send_lock:
            if self._queue_call_inflight:
                return False
            self._queue_call_inflight = True

        queue_cli = self._select_queue_client()
        if queue_cli is None:
            self.get_logger().warn(
                'Service queue_point/queue_traj_point chưa sẵn sàng.',
                throttle_duration_sec=1.0
            )
            with self._send_lock:
                self._queue_call_inflight = False
            return False
        if not self._active_queue_service_name:
            self._active_queue_service_name = getattr(queue_cli, 'srv_name', '<unknown>')
            self.get_logger().info(f'Đang stream qua service: {self._active_queue_service_name}')

        point = JointTrajectoryPoint()
        request = QueueTrajPoint.Request()
        request.joint_names = JOINT_NAMES

        if force_seed:
            # Seed point: vị trí hiện tại, v=0, t=0. Reset bộ đếm tích lũy.
            point.positions = list(self._current_joints)
            point.velocities = [0.0] * len(JOINT_NAMES)
            point.time_from_start = Duration(sec=0, nanosec=0)
            self._cumulative_time_ns = 0
        elif self._pending_point_to_resend is not None:
            # Retry điểm bị BUSY: KHÔNG tăng cumulative (timestamp đã tính từ lần trước).
            point = self._pending_point_to_resend
        elif is_hold:
            # Hold-point: gửi vị trí hiện tại nhưng với cumulative time
            # chỉ tăng nhẹ (1ms) để MotoROS2 accept mà không gây drift.
            hold_dt_ns = 1_000_000  # 1ms — minimal advance
            self._cumulative_time_ns += hold_dt_ns
            total_sec  = self._cumulative_time_ns // 1_000_000_000
            total_nsec = self._cumulative_time_ns  % 1_000_000_000
            point.positions = [float(j) for j in joints]
            point.velocities = [0.0] * len(JOINT_NAMES)
            point.time_from_start = Duration(sec=int(total_sec), nanosec=int(total_nsec))
            self._hold_point_count += 1
        else:
            # MOTION point: time_from_start TĂNG LŨY TIẾN (cumulative).
            dt_ns = int(self._queue_dt_sec * 1e9)
            self._cumulative_time_ns += dt_ns
            total_sec  = self._cumulative_time_ns // 1_000_000_000
            total_nsec = self._cumulative_time_ns  % 1_000_000_000
            point.positions = [float(j) for j in joints]
            dt = max(self._queue_dt_sec, 1e-3)
            point.velocities = [
                float((target - queued) / dt)
                for target, queued in zip(joints, self._last_queued_joints)
            ]
            point.time_from_start = Duration(sec=int(total_sec), nanosec=int(total_nsec))
            self._hold_point_count = 0  # reset hold counter

        request.point = point

        self._queue_sent_count += 1
        self._next_send_not_before_ns = self.get_clock().now().nanoseconds
        fut = queue_cli.call_async(request)
        fut.add_done_callback(lambda f, p=point, h=is_hold: self._on_queue_result(f, p, h))
        return True

    def _on_queue_result(self, future, sent_point: JointTrajectoryPoint, was_hold: bool = False):
        with self._send_lock:
            self._queue_call_inflight = False
        try:
            res = future.result()
            code = getattr(res.result_code, 'value', -1) if hasattr(res, 'result_code') else -1
            msg = getattr(res, 'message', '')
            if code == 4:
                # BUSY: giữ lại điểm để resend ở tick tiếp theo.
                # Rollback cumulative timer.
                if was_hold:
                    rollback_ns = 1_000_000  # 1ms hold
                else:
                    rollback_ns = int(self._queue_dt_sec * 1e9)
                self._cumulative_time_ns = max(0, self._cumulative_time_ns - rollback_ns)
                self._pending_point_to_resend = sent_point
                self._window_busy_count += 1
                self._window_retry_count += 1
                backoff_ns = int(self._retry_backoff_sec * 1e9)
                self._next_send_not_before_ns = self.get_clock().now().nanoseconds + backoff_ns
                self.get_logger().warn(
                    f'Queue BUSY, sẽ resend (backoff {self._retry_backoff_sec*1000:.0f}ms): msg="{msg}"',
                    throttle_duration_sec=1.0
                )
                return
            # Tương thích cả 2 biến thể firmware (SUCCESS=0 hoặc SUCCESS=1).
            if code not in (0, 1):
                self.get_logger().error(
                    f'Yaskawa TỪ CHỐI ĐIỂM! Mã lỗi (result_code): {code}, Message: "{msg}"',
                    throttle_duration_sec=1.0
                )
                self._pending_point_to_resend = None
                self._window_reject_count += 1
                return
            self._pending_point_to_resend = None
            self._last_queued_joints = list(sent_point.positions)
            self._accepted_points += 1
            self._window_ack_count += 1
            now = self.get_clock().now()
            if self._last_ack_time is not None:
                self._window_ack_interval_sum += (now - self._last_ack_time).nanoseconds / 1e9
                self._window_ack_interval_count += 1
            self._last_ack_time = now
            self._next_send_not_before_ns = now.nanoseconds
            if self._stream_state == STREAM_STATE_SEEDING:
                self._stream_state = STREAM_STATE_PREBUFFERING
                self._accepted_since_seed = 0
                self._seed_request_sent = False
                self.get_logger().info('Seed ACK nhận được, bắt đầu prebuffer.')
            elif self._stream_state == STREAM_STATE_PREBUFFERING:
                self._accepted_since_seed += 1
                if self._accepted_since_seed >= self._prebuffer_target:
                    self._stream_state = STREAM_STATE_STREAMING
                    self.get_logger().info(
                        f'Pre-buffer hoàn tất ({self._prebuffer_target} points), '
                        f'bắt đầu stream ổn định. Đang chờ target...'
                    )
            if self._accepted_points % 20 == 0:
                t_sec = sent_point.time_from_start.sec + sent_point.time_from_start.nanosec / 1e9
                self.get_logger().info(
                    f'QueueTrajPoint accepted #{self._accepted_points}: '
                    f't={t_sec:.3f}s, hold={was_hold}'
                )
            if self._queue_debug_log_count < 5:
                self._queue_debug_log_count += 1
                t_sec = sent_point.time_from_start.sec + sent_point.time_from_start.nanosec / 1e9
                self.get_logger().info(
                    f'Queue accepted sample#{self._queue_debug_log_count}: '
                    f'svc={self._active_queue_service_name}, '
                    f't_cumul={t_sec:.4f}s, code={code}, msg="{msg}", '
                    f'hold={was_hold}'
                )
        except Exception as e:
            self.get_logger().error(f'Lỗi khi nhận phản hồi từ queue_traj_point: {e}')

    def _select_queue_client(self):
        if self._queue_point_cli.wait_for_service(timeout_sec=0.01):
            return self._queue_point_cli
        if self._queue_point_cli_alt.wait_for_service(timeout_sec=0.01):
            return self._queue_point_cli_alt
        if self._queue_point_cli_alt2.wait_for_service(timeout_sec=0.01):
            return self._queue_point_cli_alt2
        return None

    def _log_runtime_rates(self):
        now = self.get_clock().now()
        elapsed = (now - self._rate_window_start).nanoseconds / 1e9
        if elapsed < 5.0:
            return
        tick_hz = self._tick_count / elapsed
        queue_send_hz = self._queue_sent_count / elapsed
        ack_hz = self._window_ack_count / elapsed
        busy_hz = self._window_busy_count / elapsed
        inter_ack_ms = (
            (self._window_ack_interval_sum / self._window_ack_interval_count) * 1000.0
            if self._window_ack_interval_count > 0 else 0.0
        )
        cumul_sec = self._cumulative_time_ns / 1e9
        self.get_logger().info(
            'Runtime rate: '
            f'state={self._stream_state}, '
            f'tick_hz={tick_hz:.1f}, queue_send_hz={queue_send_hz:.1f}, '
            f'ack_hz={ack_hz:.1f}, busy_hz={busy_hz:.1f}, '
            f'retry_count={self._window_retry_count}, '
            f'reject_count={self._window_reject_count}, '
            f'inter_ack_ms={inter_ack_ms:.1f}, '
            f'max_joint_delta={self._window_max_joint_delta:.3f}, '
            f'cumul_time={cumul_sec:.2f}s, '
            f'hold_count={self._hold_point_count}'
        )
        if self._last_target_update_ns > 0:
            target_age_s = (now.nanoseconds - self._last_target_update_ns) / 1e9
            if target_age_s > 2.0:
                self.get_logger().warn(
                    f'Target stream stale: không có target mới trong {target_age_s:.1f}s'
                )
        self._tick_count = 0
        self._queue_sent_count = 0
        self._rate_window_start = now
        self._window_ack_count = 0
        self._window_busy_count = 0
        self._window_retry_count = 0
        self._window_reject_count = 0
        self._window_max_joint_delta = 0.0
        self._window_ack_interval_sum = 0.0
        self._window_ack_interval_count = 0

    def _check_no_motion_watchdog(self):
        if self._accepted_points < NO_MOTION_MIN_ACCEPTED_POINTS:
            return
        # Chỉ cảnh báo nếu đang gửi motion points (không phải hold)
        if self._hold_point_count > 0:
            return
        now = self.get_clock().now()
        since_motion = (now - self._last_motion_time).nanoseconds / 1e9
        since_warn = (now - self._last_warn_time).nanoseconds / 1e9
        if since_motion >= NO_MOTION_WARN_SEC and since_warn >= NO_MOTION_WARN_SEC:
            self._last_warn_time = now
            self.get_logger().warn(
                'QueueTrajPoint đã được accept nhưng joint_states hầu như không đổi. '
                'Kiểm tra: 1) Robot ở mode AUTO? 2) Servo ON? 3) E-stop cleared?'
            )

# ═══════════════════════════════════════════════════════════════════
# DEMO NODE: Test Cartesian trajectory
# ═══════════════════════════════════════════════════════════════════

class CartesianDemoPublisher(Node):
    """
    Node test: publish các điểm Cartesian theo pattern.

    Dùng:
        python3 cartesian_streamer.py --demo circle
        python3 cartesian_streamer.py --demo line
        python3 cartesian_streamer.py --demo lissajous
    """

    def __init__(self, mode: str = 'line', omega: float = 0.5, amplitude: float = 0.05, stream_period_sec: float = 1.0/DEFAULT_STREAM_HZ):
        super().__init__('cartesian_demo')
        self._mode = mode
        self._omega = omega          # rad/s — tốc độ góc
        self._amplitude = amplitude  # mét — biên độ chuyển động
        self._stream_period = stream_period_sec
        self._t = 0.0

        self._pub = self.create_publisher(
            Float64MultiArray, '/cartesian_streamer/target_xyz', 10)

        # Đọc EE pose hiện tại để biết điểm xuất phát
        self._base_x = 0.0
        self._base_y = 0.0
        self._base_z = 0.0

        self._ee_sub = self.create_subscription(
            PoseStamped, '/cartesian_streamer/current_pose',
            self._on_ee, 10)

        self._got_base = False
        # Timer chưa bắt đầu — chỉ chuyển động SAU KHI có EE feedback
        self._timer = None
        self._start_time = None

        period_sec = 2.0 * math.pi / self._omega
        self.get_logger().info(
            f'CartesianDemo [{mode}] khởi động:\n'
            f'  omega={omega:.2f} rad/s (period={period_sec:.1f}s)\n'
            f'  amplitude={amplitude*100:.1f} cm\n'
            f'  Đợi EE feedback trước khi chạy...'
        )

    def _on_ee(self, msg: PoseStamped):
        if not self._got_base:
            self._base_x = msg.pose.position.x
            self._base_y = msg.pose.position.y
            self._base_z = msg.pose.position.z
            self._got_base = True
            self.get_logger().info(
                f'Base EE: ({self._base_x:.4f}, '
                f'{self._base_y:.4f}, {self._base_z:.4f})')
            # Bắt đầu publish target SAU KHI biết vị trí thật
            if self._timer is None:
                self._start_time = self.get_clock().now()
                self._timer = self.create_timer(self._stream_period, self._tick)
                self.get_logger().info('▶ Bắt đầu demo pattern!')

    def _tick(self):
        if not self._got_base:
            return  # chưa có vị trí gốc
        self._t += self._stream_period  # step khớp với stream rate
        amp = self._amplitude
        w = self._omega

        if self._mode == 'line':
            # Tiến lùi trên trục X
            x = self._base_x + amp * math.sin(w * self._t)
            y = self._base_y
            z = self._base_z

        elif self._mode == 'circle':
            # Vòng tròn trên mặt phẳng YZ
            x = self._base_x
            y = self._base_y + amp * math.cos(w * self._t)
            z = self._base_z + amp * math.sin(w * self._t)

        elif self._mode == 'lissajous':
            # Đường Lissajous trên mặt phẳng XY
            x = self._base_x + amp * math.sin(2 * w * self._t)
            y = self._base_y + amp * math.sin(w * self._t + math.pi / 4)
            z = self._base_z

        else:
            return

        msg = Float64MultiArray()
        msg.data = [x, y, z]
        self._pub.publish(msg)
        self.get_logger().info(
            f'[{self._mode}] t={self._t:.2f}s → ({x:.4f}, {y:.4f}, {z:.4f})',
            throttle_duration_sec=1.0)


# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description='Cartesian Streamer cho MotoROS2 Point Queue Mode',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ví dụ:
  # Test cơ bản: di chuyển tuyến tính trên trục X, 5cm, chậm
  python3 cartesian_streamer.py --demo line

  # Vòng tròn nhanh hơn, biên độ lớn hơn
  python3 cartesian_streamer.py --demo circle --omega 1.0 --amplitude 0.08

  # Lissajous pattern
  python3 cartesian_streamer.py --demo lissajous --omega 0.3

  # Chỉ chạy streamer, chờ target từ AI/Camera node
  python3 cartesian_streamer.py

  # Tune queue parameters
  python3 cartesian_streamer.py --demo line --queue-dt 0.06 --retry-backoff-ms 20
""")
    parser.add_argument(
        '--stream-hz', type=float, default=DEFAULT_STREAM_HZ,
        help=f'Tần số stream tick (Hz) [default: {DEFAULT_STREAM_HZ}]')
    parser.add_argument(
        '--queue-dt', type=float, default=None,
        help=f'time delta giữa mỗi motion point (giây) [default: 1/stream_hz]')
    parser.add_argument(
        '--prebuffer', type=int, default=QUEUE_PREBUFFER_POINTS,
        help=f'số điểm prebuffer trước khi vào streaming [default: {QUEUE_PREBUFFER_POINTS}]')
    parser.add_argument(
        '--retry-backoff-ms', type=float, default=QUEUE_RETRY_BACKOFF_SEC * 1000.0,
        help=f'backoff (ms) khi queue trả BUSY [default: {QUEUE_RETRY_BACKOFF_SEC*1000:.0f}]')
    parser.add_argument(
        '--demo', choices=['circle', 'line', 'lissajous'],
        default=None,
        help='Chạy demo pattern (không cần AI node ngoài)')
    parser.add_argument(
        '--omega', type=float, default=0.5,
        help='Tốc độ góc cho demo pattern (rad/s) [default: 0.5]')
    parser.add_argument(
        '--amplitude', type=float, default=0.05,
        help='Biên độ cho demo pattern (mét) [default: 0.05]')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    executor = MultiThreadedExecutor(num_threads=4)

    stream_hz = max(args.stream_hz, 1.0)
    # queue_dt defaults to 1/stream_hz if not explicitly set
    queue_dt = args.queue_dt if args.queue_dt is not None else (1.0 / stream_hz)
    queue_dt = max(queue_dt, 0.01)

    streamer = CartesianStreamer(
        stream_hz=stream_hz,
        queue_dt_sec=queue_dt,
        prebuffer_points=max(args.prebuffer, 1),
        retry_backoff_sec=max(args.retry_backoff_ms, 0.0) / 1000.0,
    )
    executor.add_node(streamer)

    if args.demo:
        demo = CartesianDemoPublisher(
            mode=args.demo,
            omega=args.omega,
            amplitude=args.amplitude,
            stream_period_sec=1.0 / stream_hz,
        )
        executor.add_node(demo)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        streamer.get_logger().info('Shutdown.')
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()