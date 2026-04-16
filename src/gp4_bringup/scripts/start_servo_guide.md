# Hướng dẫn chạy Cartesian Streamer — MotoROS2 Point Queue Mode

## Tổng quan

`cartesian_streamer.py` stream các điểm joint trajectory liên tục xuống robot Yaskawa GP4 thông qua chế độ **Point Queue Mode** của MotoROS2. Quy trình:

1. Nhận tọa độ Cartesian (XYZ hoặc Pose) từ AI/Camera node hoặc demo pattern
2. Giải IK qua MoveIt (`/compute_ik`)
3. Gửi joint positions xuống robot qua service `/yaskawa/queue_traj_point`

> **Lưu ý**: KHÔNG chạy `real_robot.launch.py` — file đó bật `ros2_control` + `gp4_arm_controller`, tạo đường lệnh thứ hai xung đột với MotoROS2 queue mode.

---

## Bước 1: Chuẩn bị robot

Trên pendant Yaskawa:
- Chuyển sang mode **AUTO** (không phải TEACH)
- Đảm bảo **E-Stop** đã cleared
- **Servo ON** (script sẽ tự gọi, nhưng kiểm tra lại nếu có lỗi)

## Bước 2: Khởi động MoveIt + Robot State Publisher

```bash
# Terminal 1
cd ~/Downloads/gp4_ws
source install/setup.bash
ros2 launch gp4_moveit_config gp4_start.launch.py
```

Chờ cho đến khi thấy:
- `move_group` đã sẵn sàng
- `/yaskawa/joint_states` đang publish
- RViz hiển thị robot

## Bước 3: Chạy Cartesian Streamer

```bash
# Terminal 2
cd ~/Downloads/gp4_ws
source install/setup.bash
python3 src/gp4_bringup/scripts/cartesian_streamer.py --demo circle
```

### Xác nhận hoạt động

Log phải hiển thị tuần tự:
```
[INFO] Nhận joint_states: [...]              ← Đã kết nối robot
[INFO] Đang gọi reset_error...               ← Chuỗi khởi động
[INFO] Đang gọi servo_on...
[INFO] Gọi stop_traj_mode...
[INFO] Bật StartPointQueueMode...
[INFO] ✓ Point Queue Mode active.             ← Queue mode OK
[INFO] Bootstrap EE pose via FK: (x, y, z)    ← Biết vị trí ban đầu
[INFO] Seed ACK nhận được, bắt đầu prebuffer. ← Seed point accepted
[INFO] Pre-buffer hoàn tất, bắt đầu stream... ← Bắt đầu streaming
[INFO] IK OK → Δmax=0.xxxx rad, joints: [...] ← Robot BẮT ĐẦU DI CHUYỂN
```

---

## Các chế độ Demo

| Mode | Mô tả | Command |
|---|---|---|
| `line` | Tiến lùi trên trục X | `--demo line` |
| `circle` | Vòng tròn trên mặt phẳng YZ | `--demo circle` |
| `lissajous` | Đường Lissajous trên mặt phẳng XY | `--demo lissajous` |
| *(không có)* | Chờ target từ AI/Camera node | *(không đặt --demo)* |

---

## Tham số có thể điều chỉnh

### Tham số stream

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `--stream-hz` | `20` | Tần số gửi điểm (Hz). Timer chạy ở tần số này. **Đây là tần số thực tế gửi point xuống robot**. |
| `--queue-dt` | `1/stream_hz` | Khoảng cách thời gian (giây) giữa mỗi motion point trong trajectory. Mặc định bằng `1/stream_hz`. MotoROS2 dùng giá trị này để tính tốc độ di chuyển giữa 2 điểm. |
| `--prebuffer` | `3` | Số điểm gửi trước (hold-points) để MotoROS2 có buffer sẵn trước khi bắt đầu stream thật. |
| `--retry-backoff-ms` | `15` | Thời gian chờ (ms) trước khi gửi lại khi nhận BUSY. |

### Tham số demo pattern

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `--omega` | `0.5` | Tốc độ góc (rad/s). Xác định tốc độ di chuyển theo pattern. `omega=0.5` → period = 12.6 giây. `omega=1.0` → period = 6.3 giây. |
| `--amplitude` | `0.05` | Biên độ chuyển động (mét). `0.05` = 5cm. Tăng lên `0.08`–`0.10` cho chuyển động rõ hơn. |

### Hằng số trong code (sửa trực tiếp nếu cần)

| Hằng số | Giá trị | Ý nghĩa |
|---|---|---|
| `SMOOTH_ALPHA` | `0.6` | Hệ số smooth (0.0–1.0). Cao hơn = phản hồi nhanh hơn, thấp hơn = mượt hơn. |
| `MAX_JOINT_DELTA` | `0.5 rad` | Bước nhảy joint tối đa cho phép mỗi điểm. Nếu IK cho kết quả nhảy quá xa, điểm bị bỏ qua để bảo vệ robot. |
| `IK_TIMEOUT_SEC` | `0.2 s` | Timeout cho mỗi lần gọi IK. |
| `WS_X/Y/Z` | `±0.7m / 0.05–0.8m` | Biên workspace an toàn. Điểm ngoài vùng này bị từ chối. |

---

## Ví dụ lệnh

```bash
# Mặc định: circle 20Hz, chậm, an toàn
python3 src/gp4_bringup/scripts/cartesian_streamer.py --demo circle

# Line pattern, tốc độ vừa phải
python3 src/gp4_bringup/scripts/cartesian_streamer.py --demo line --omega 0.5

# Circle nhanh hơn, biên độ lớn hơn
python3 src/gp4_bringup/scripts/cartesian_streamer.py --demo circle --omega 1.0 --amplitude 0.08

# Giảm tần số xuống 15Hz nếu thấy BUSY quá nhiều
python3 src/gp4_bringup/scripts/cartesian_streamer.py --demo circle --stream-hz 15

# Tăng tần số lên 22Hz (gần trần robot)
python3 src/gp4_bringup/scripts/cartesian_streamer.py --demo circle --stream-hz 22

# Chỉ chạy streamer, chờ target từ node khác
python3 src/gp4_bringup/scripts/cartesian_streamer.py
```

---

## Đọc hiểu log Runtime Rate

```
Runtime rate: state=streaming, tick_hz=20.0, queue_send_hz=19.8,
ack_hz=19.5, busy_hz=0.3, retry_count=2, reject_count=0,
inter_ack_ms=51.2, max_joint_delta=0.015, cumul_time=10.50s, hold_count=0
```

| Trường | Ý nghĩa | Giá trị tốt |
|---|---|---|
| `state` | Trạng thái hiện tại | `streaming` |
| `tick_hz` | Tần số timer thực tế | Bằng `--stream-hz` |
| `queue_send_hz` | Số điểm gửi/giây | Gần bằng `tick_hz` |
| `ack_hz` | Số điểm robot accept/giây | Gần bằng `queue_send_hz` |
| `busy_hz` | Số lần bị BUSY/giây | **< 1.0** (càng thấp càng tốt) |
| `retry_count` | Tổng retry trong window 5s | **< 5** |
| `reject_count` | Điểm bị từ chối (lỗi) | **= 0** |
| `inter_ack_ms` | Thời gian xử lý trung bình/điểm | ~43–50ms cho GP4 |
| `max_joint_delta` | Bước nhảy joint lớn nhất (rad) | **> 0** (= robot đang di chuyển) |
| `cumul_time` | Thời gian tích lũy đã gửi | Tăng đều |
| `hold_count` | Số hold-points liên tiếp | `0` khi đang có target |

### Chẩn đoán nhanh

| Triệu chứng | Nguyên nhân | Cách sửa |
|---|---|---|
| `max_joint_delta=0.000` | Không có target / robot không ở AUTO | Thêm `--demo`, kiểm tra pendant |
| `busy_hz > 3` | Gửi nhanh hơn robot xử lý | Giảm `--stream-hz` |
| `reject_count > 0` | Lỗi joint names hoặc queue mode | Kiểm tra JOINT_NAMES, restart queue mode |
| `hold_count` tăng liên tục | Không có target hoặc IK fail | Kiểm tra topic target, workspace limits |
| `ack_hz ≈ 0` trong streaming | Service bị mất kết nối | Kiểm tra micro-ROS agent, kết nối mạng |

---

## Giao tiếp với Cartesian Streamer từ node khác

### Gửi target XYZ (đơn giản nhất)

```bash
# Gửi 1 điểm XYZ qua command line
ros2 topic pub --once /cartesian_streamer/target_xyz \
  std_msgs/msg/Float64MultiArray "{data: [0.3, 0.2, 0.4]}"
```

### Gửi target Pose đầy đủ

```bash
ros2 topic pub --once /cartesian_streamer/target_pose \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, \
    pose: {position: {x: 0.3, y: 0.2, z: 0.4}, \
           orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"
```

### Đọc feedback vị trí EE hiện tại

```bash
ros2 topic echo /cartesian_streamer/current_pose
```

---

## Thông số robot đã đo được

| Thông số | Giá trị |
|---|---|
| Tần số xử lý thực tế (inter_ack_ms) | ~43–45 ms/point |
| Tần số accept tối đa | **~22–23 Hz** |
| Tần số gửi khuyến nghị | **20 Hz** (margin an toàn) |
| BUSY rate ở 20Hz | < 2% |
| BUSY rate ở 25Hz | ~6–10% |
