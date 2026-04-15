#!/usr/bin/env python3
"""
cartesian_streamer.py
─────────────────────
Nhận tọa độ Cartesian (PoseStamped) từ AI/Camera node,
giải IK tại 30Hz, và stream joint angles xuống MotoROS2.

Quy trình khởi động (2 terminal):
  Terminal 1: ros2 launch gp4_moveit_config gp4_start.launch.py
              (khởi động: move_group, RViz, robot_state_publisher, restamp_joint_states)
  Terminal 2: python3 cartesian_streamer.py

Lưu ý kiến trúc:
  KHÔNG chạy real_robot.launch.py — file đó bật ros2_control + gp4_arm_controller
  tạo con đường lệnh thứ hai (xung đột) xuống robot.
  MotoROS2 driver chạy trực tiếp trên YRC1000micro và tự expose /yaskawa/* services.

AI node gửi lệnh qua:
  /cartesian_streamer/target_pose  → geometry_msgs/PoseStamped
  hoặc
  /cartesian_streamer/target_xyz   → Float64MultiArray [x, y, z]
                                     (orientation giữ nguyên từ vị trí hiện tại)

Test tích hợp:
  python3 cartesian_streamer.py --demo circle      # vẽ vòng tròn
  python3 cartesian_streamer.py --demo line        # tiến lùi trên trục X
  python3 cartesian_streamer.py --demo lissajous   # đường Lissajous
"""

import sys
import math
import threading
import argparse

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
GROUP_NAME   = 'gp4_arm'
EE_LINK      = 'tool0'
BASE_FRAME   = 'base_link'

# Workspace an toàn (mét) — điều chỉnh theo môi trường thực tế
WS_X = (-0.7,  0.7)
WS_Y = (-0.7,  0.7)
WS_Z = ( 0.05, 0.8)

# Thời gian mỗi điểm (30 Hz)
POINT_DURATION_SEC = 1.0 / 30.0

# IK timeout
IK_TIMEOUT_SEC = 0.2

# Smooth: tỉ lệ tiến về target mỗi tick (0.0–1.0)
SMOOTH_ALPHA = 0.5       # Tăng lên để phản hồi nhanh hơn

# An toàn: bước nhảy joint tối đa cho phép mỗi điểm (rad)
MAX_JOINT_DELTA = 0.5    # Tăng lên để cho phép di chuyển rõ rệt trong 1 giây

# Debug watchdog: nếu queue point được accept nhưng joint gần như đứng yên
NO_MOTION_WARN_SEC = 3.0
NO_MOTION_EPS_RAD = 1e-3
NO_MOTION_MIN_ACCEPTED_POINTS = 6


class CartesianStreamer(Node):

    def __init__(self):
        super().__init__('cartesian_streamer')

        self._cb = ReentrantCallbackGroup()

        # ── State ────────────────────────────────────────────────
        self._current_joints: list[float] = [0.0] * 6
        self._got_joints = False
        self._queue_mode_active = False
        self._first_queue_point_sent = False
        self._queue_call_inflight = False
        self._accepted_points = 0
        self._queue_debug_log_count = 0
        self._active_queue_service_name = ''
        self._last_motion_time = self.get_clock().now()
        self._last_warn_time = self.get_clock().now()
        self._tick_count = 0
        self._queue_sent_count = 0
        self._rate_window_start = self.get_clock().now()

        # Target Cartesian pose (được smooth từng bước)
        self._target_pose: Pose | None = None
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
        # Stream rate = 10Hz (khớp với POINT_DURATION_SEC=0.1)
        self._stream_timer = self.create_timer(
            POINT_DURATION_SEC, self._stream_tick, callback_group=self._cb)

        self._startup_timer = self.create_timer(
            0.5, self._startup, callback_group=self._cb)

        self.get_logger().info(
            'CartesianStreamer khởi động.\n'
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
                self._first_queue_point_sent = False
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
    # STREAM LOOP 10Hz
    # ═══════════════════════════════════════════════════════════════

    def _stream_tick(self):
        try:
            if not self._queue_mode_active or not self._got_joints:
                return

            self._tick_count += 1
            self._log_runtime_rates()
            self._check_no_motion_watchdog()

            # ── Bước 0: Khởi tạo ─────────────────────────────────────
            if not self._first_queue_point_sent:
                # Bootstrap initial EE pose via FK
                initial_pose = self._solve_fk_sync(list(self._current_joints))
                if initial_pose:
                    self._current_ee_pose = initial_pose
                    fb = PoseStamped()
                    fb.header.frame_id = BASE_FRAME
                    fb.header.stamp = self.get_clock().now().to_msg()
                    fb.pose = initial_pose
                    self._ee_pub.publish(fb)
                    self.get_logger().info('Bootstrap EE pose via FK successful.')

                # Quy tắc queue mode: điểm đầu tiên phải đúng trạng thái hiện tại, t=0, v=0.
                if self._send_joint_point(list(self._current_joints), force_seed=True):
                    self._first_queue_point_sent = True
                    self.get_logger().info('Đã gửi first queue point (current joints).')
                return

            if self._target_pose is None:
                return   # Chưa có lệnh — đứng yên

            # ── Bước 1: Smooth pose (interpolate về target) ──────────
            smoothed = self._smooth_pose(self._target_pose)

            # ── Bước 2: Giải IK ──────────────────────────────────────
            joint_solution = self._solve_ik_sync(smoothed)

            if joint_solution is None:
                # IK thất bại → giữ vị trí cũ
                self._ik_fail_count += 1
                if self._ik_fail_count % 10 == 1:
                    self.get_logger().warn(
                        f'IK thất bại {self._ik_fail_count} lần liên tiếp. '
                        'Robot giữ nguyên vị trí.')
                return

            # ── An toàn: kiểm tra bước nhảy joint ────────────────────
            max_delta = max(abs(j - c) for j, c in
                            zip(joint_solution, self._current_joints))
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
                f'IK OK → joints: [{joint_solution[0]:.3f}, {joint_solution[1]:.3f}, ...]',
                throttle_duration_sec=2.0)

            # ── Bước 3: Gửi xuống robot ──────────────────────────────
            self._send_joint_point(joint_solution)

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
        Tránh dùng rclpy.spin_until_future_complete() bên trong callback vì
        nó cạnh tranh với executor thread và có thể gây deadlock.
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

        # Seed = vị trí hiện tại (đây là lý do IK nhanh)
        seed = RobotState()
        seed.joint_state.name     = JOINT_NAMES
        seed.joint_state.position = list(self._current_joints)
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

            # Chờ tối đa IK_TIMEOUT_SEC + buffer nhỏ, không block executor
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

    def _send_joint_point(self, joints: list[float], force_seed: bool = False):
        if self._queue_call_inflight:
            return False
        queue_cli = self._select_queue_client()
        if queue_cli is None:
            self.get_logger().warn(
                'Service queue_point/queue_traj_point chưa sẵn sàng.',
                throttle_duration_sec=1.0
            )
            return False
        if not self._active_queue_service_name:
            self._active_queue_service_name = getattr(queue_cli, 'srv_name', '<unknown>')
            self.get_logger().info(f'Đang stream qua service: {self._active_queue_service_name}')

        point = JointTrajectoryPoint()
        request = QueueTrajPoint.Request()
        request.joint_names = JOINT_NAMES

        if force_seed:
            point.positions = list(self._current_joints)
            point.velocities = [0.0] * len(JOINT_NAMES)
            point.time_from_start = Duration(sec=0, nanosec=0)
        else:
            point.positions = [float(j) for j in joints]
            dt = max(POINT_DURATION_SEC, 1e-3)
            point.velocities = [
                float((target - current) / dt)
                for target, current in zip(joints, self._current_joints)
            ]
            sec = int(dt)
            nanosec = int((dt - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        request.point = point

        self._queue_call_inflight = True
        self._queue_sent_count += 1
        fut = queue_cli.call_async(request)
        fut.add_done_callback(self._on_queue_result)
        return True

    def _on_queue_result(self, future):
        self._queue_call_inflight = False
        try:
            res = future.result()
            # SUCCESS của QueueResultEnum là 1, không phải 0.
            if res.result_code.value != 1:
                self.get_logger().error(
                    f'Robot TỪ CHỐI điểm queue: code={res.result_code.value}, msg="{res.message}"',
                    throttle_duration_sec=1.0
                )
                return
            self._accepted_points += 1
            if self._accepted_points % 20 == 0:
                self.get_logger().info(f'QueueTrajPoint accepted count={self._accepted_points}')
            if self._queue_debug_log_count < 5:
                self._queue_debug_log_count += 1
                self.get_logger().info(
                    f'Queue accepted sample#{self._queue_debug_log_count}: '
                    f'svc={self._active_queue_service_name}, '
                    f't0={request_time_hint()}'
                )
        except Exception as e:
            self.get_logger().error(f'Lỗi khi gọi queue_traj_point: {e}')

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
        self.get_logger().info(
            f'Runtime rate: tick_hz={tick_hz:.1f}, queue_send_hz={queue_send_hz:.1f}'
        )
        self._tick_count = 0
        self._queue_sent_count = 0
        self._rate_window_start = now

    def _check_no_motion_watchdog(self):
        if self._accepted_points < NO_MOTION_MIN_ACCEPTED_POINTS:
            return
        now = self.get_clock().now()
        since_motion = (now - self._last_motion_time).nanoseconds / 1e9
        since_warn = (now - self._last_warn_time).nanoseconds / 1e9
        if since_motion >= NO_MOTION_WARN_SEC and since_warn >= NO_MOTION_WARN_SEC:
            self._last_warn_time = now
            self.get_logger().warn(
                'QueueTrajPoint đã được accept nhưng joint_states hầu như không đổi. '
            )

def request_time_hint():
    return f"{POINT_DURATION_SEC:.4f}s"


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

    def __init__(self, mode: str = 'line'):
        super().__init__('cartesian_demo')
        self._mode = mode
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

        self.get_logger().info(
            f'CartesianDemo [{mode}] khởi động. Đợi EE feedback trước khi chạy...')

    def _on_ee(self, msg: PoseStamped):
        if not self._got_base:
            self._base_x = msg.pose.position.x
            self._base_y = msg.pose.position.y
            self._base_z = msg.pose.position.z
            self._got_base = True
            self.get_logger().info(
                f'Base EE: ({self._base_x:.3f}, '
                f'{self._base_y:.3f}, {self._base_z:.3f})')
            # Bắt đầu publish target SAU KHI biết vị trí thật
            if self._timer is None:
                self._timer = self.create_timer(POINT_DURATION_SEC, self._tick)
                self.get_logger().info('Bắt đầu demo pattern!')

    def _tick(self):
        if not self._got_base:
            return  # chưa có vị trí gốc
        self._t += POINT_DURATION_SEC  # step khớp với stream rate

        if self._mode == 'line':
            # Tiến lùi trên trục X, biên độ 5cm
            x = self._base_x + 0.05 * math.sin(self._t)
            y = self._base_y
            z = self._base_z

        elif self._mode == 'circle':
            # Vòng tròn trên mặt phẳng YZ, bán kính 5cm
            x = self._base_x
            y = self._base_y + 0.05 * math.cos(self._t)
            z = self._base_z + 0.05 * math.sin(self._t)

        elif self._mode == 'lissajous':
            # Đường Lissajous trên mặt phẳng XY
            x = self._base_x + 0.06 * math.sin(2 * self._t)
            y = self._base_y + 0.06 * math.sin(self._t + math.pi / 4)
            z = self._base_z

        else:
            return

        msg = Float64MultiArray()
        msg.data = [x, y, z]
        self._pub.publish(msg)
        self.get_logger().info(
            f'[{self._mode}] target: ({x:.4f}, {y:.4f}, {z:.4f})',
            throttle_duration_sec=0.5)


# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--demo', choices=['circle', 'line', 'lissajous'],
        default=None,
        help='Chạy demo pattern (không cần AI node ngoài)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    executor = MultiThreadedExecutor(num_threads=4)

    streamer = CartesianStreamer()
    executor.add_node(streamer)

    if args.demo:
        demo = CartesianDemoPublisher(mode=args.demo)
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
