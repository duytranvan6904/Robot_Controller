# Hướng Dẫn Điều Khiển Quỹ Đạo Liên Tục (Streaming) GP4

Tài liệu này hướng dẫn cách vận hành hệ thống điều khiển liên tục (Real-time Streaming) cho tay máy Yaskawa GP4 sử dụng **MoveIt Servo** kết hợp với **MotoROS2**. 

Kiến trúc này được tối ưu cho các ứng dụng yêu cầu độ phản hồi cao như:
- Điều khiển bằng AI/Camera (Visual Servoing).
- Teleoperation (Điều khiển từ xa bằng Joystick/Cảm biến).
- Tránh vật cản động thông qua sensor.

---

## 🏗 KIẾN TRÚC HỆ THỐNG

Luồng dữ liệu điều khiển được thực hiện như sau:
1. **AI/User App**: Gửi lệnh vận tốc (Twist) tại tần số 30Hz - 100Hz.
2. **MoveIt Servo**: Nhận Twist, tính toán động học ngược (IK) và nội suy mượt mà để tránh va chạm/singularity.
3. **Servo Bridge**: Chuyển đổi quỹ đạo từ MoveIt Servo thành các điểm lẻ để gửi vào hàng đợi của Robot.
4. **MotoROS2 (Robot)**: Thực thi các điểm trong hàng đợi với độ trễ tối thiểu.

---

## 🚀 QUY TRÌNH KHỞI ĐỘNG (5 Terminal)

Để chạy hệ thống trên robot thật, hãy thực hiện theo thứ tự sau:
```bash
cd ~/Downloads/gp4_ws
source install/setup.bash
```

### Terminal 1: Kết nối Robot (Hardware Interface)
Khởi động kết nối vật lý với tủ điện YRC1000micro.
```bash
ros2 launch gp4_moveit_config real_robot.launch.py robot_ip:=192.168.1.33
```
*Đảm bảo controller_manager báo "joint_state_broadcaster" và "gp4_arm_controller" đã Active.*

### Terminal 2: MoveIt Core & RViz
Khởi động bộ não MoveIt để quản lý mô hình và va chạm.
```bash
ros2 launch gp4_moveit_config gp4_start.launch.py
```

### Terminal 3: MoveIt Servo
Khởi động servo trong thực tế:
```bash
ros2 service call /yaskawa/start_point_queue_mode motoros2_interfaces/srv/StartPointQueueMode "{}"
```

Khởi động node nội suy thời gian thực.
```bash
ros2 launch gp4_moveit_config gp4_servo.launch.py
```

### Terminal 4: Servo Bridge
Node trung gian chuyển tiếp dữ liệu từ Servo sang Robot.
```bash
python3 src/gp4_bringup/scripts/servo_bridge.py
```

### Terminal 5: Script Test/AI 
Chạy kịch bản điều khiển thực tế (ví dụ: robot di chuyển hình sin).
```bash
python3 src/gp4_bringup/scripts/test_continuous_move.py
```

---

## 💡 LƯU Ý KỸ THUẬT & AN TOÀN

### 1. Cơ chế nội suy (Interpolation)
Khác với việc lập trình điểm-điểm (PTP) thông thường, hệ thống này liên tục "nhồi" các điểm vào robot. 
- Nếu bạn ngừng gửi lệnh, Robot sẽ dừng lại tại vị trí hiện tại.
- MoveIt Servo sẽ tự động tính toán để robot dừng lại an toàn nếu sắp va chạm hoặc chạm giới hạn khớp.

### 2. Kiểm soát vận tốc
Trong file `test_continuous_move.py`, vận tốc được quy định bởi các thông số:
```python
msg.twist.linear.x = 0.03  # Tốc độ 3cm/s theo trục X
```
> [!WARNING]
> Luôn bắt đầu với vận tốc thấp (v < 0.05 m/s) khi thử nghiệm thuật toán AI mới.

### 3. Nút dừng khẩn cấp (E-Stop)
- Luôn giữ Teach Pendant trên tay.
- Nếu thấy robot rung lắc hoặc di chuyển lạ, nhấn **E-Stop** ngay lập tức.
- Bạn cũng có thể tắt Terminal 5 (Ctrl+C), script `test_continuous_move.py` đã được lập trình để gửi lệnh vận tốc bằng 0 khi thoát.

### 4. Xử lý lỗi "Tolerance Exceeded"
Nếu trên Terminal 1 xuất hiện cảnh báo trễ, hãy kiểm tra:
- Tần suất gửi lệnh của AI có ổn định không (nên dùng Timer 30Hz).
- Tải của CPU (nếu AI quá nặng làm trễ luồng gửi lệnh).

---

## 🛠 TÙY CHỈNH CHO PROJECT AI

Để tích hợp vào Code AI của bạn, hãy sử dụng đoạn mã mẫu sau:

```python
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger

# 1. Start Servo (chỉ gọi 1 lần khi bắt đầu)
start_client = node.create_client(Trigger, '/servo_node/start_servo')
start_client.call_async(Trigger.Request())

# 2. Publish Twist (gọi liên tục trong vòng lặp AI/Camera)
twist_pub = node.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

msg = TwistStamped()
msg.header.frame_id = 'base_link' 
msg.twist.linear.x = camera_delta_x * gain
msg.twist.linear.y = camera_delta_y * gain
twist_pub.publish(msg)
```
