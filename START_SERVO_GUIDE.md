# Hướng Dẫn Khởi Chạy GP4 MoveIt Servo (Streaming 30Hz)

Tài liệu này hướng dẫn bạn cách khởi chạy hệ thống nội suy thời gian thực (MoveIt Servo) trên GP4 và cách gửi điểm tọa độ vận tốc liên tục. Tính năng này được dùng cho các Camera AI hoặc các bộ điều khiển tay cầm từ xa.

---

## 1. Mở Terminal 1: Khởi động Lõi Nội Suy và Mô Phỏng
Lệnh này sẽ bật RViz, giao diện đồ họa, bộ giả lập vật lý và đặc biệt là bộ **Servo Node**. Servo Node này đóng vai trò nội suy động học ngược ở tần số cực cao (lên đến 100Hz) mà không cần chờ OMPL lập kế hoạch.

```bash
cd ~/Downloads/gp4_ws
source install/setup.bash
ros2 launch gp4_bringup sim.launch.py
```

*Đợi màn hình RViz hiện lên và có thông báo hệ thống đã khởi động xong.*

---

## 2. Mở Terminal 2: Khởi chạy Bài Test Chuyển Động Liên Tục
Để chắc chắn hệ thống không có trục trặc hay đứng im, chúng ta cần sinh ra các điểm trích xuất cách nhau một đoạn vi ly. File Node Python này là một ví dụ mẫu của thuật toán sinh tọa độ ảo 30Hz:

```bash
cd ~/Downloads/gp4_ws
source install/setup.bash
python3 src/gp4_bringup/test_servo_sine_wave.py
```

*Lúc này bạn sẽ thấy ở màn hình RViz, cánh tay đang đưa qua đưa lại êm ru (đi tới và đi lùi 2cm/s theo biên độ 2 giây đổi hướng).*

---

## 3. Tích Hợp Vào Thuật Toán Camera AI của Bạn
Servo đã được cấu hình nghe data trên Node topic sau:
- **Tên Topic**: `/servo_node_main/delta_twist_cmds`
- **Loại Data**: `geometry_msgs/msg/TwistStamped`
- **Tần suất khuyên dùng**: 30 - 50 Hz

Khi code bằng Python (hoặc ROS 2 C# trên màn hình Windows) ở luồng AI Pipeline để điều khiển thật, lệnh cốt lõi của bạn chỉ đơn giản là:

```python
from geometry_msgs.msg import TwistStamped

# 1. Tạo biến msg
msg = TwistStamped()
msg.header.frame_id = 'base_link'

# 2. Truyền Vận tốc đầu đo trên trục X, Y, Z
msg.twist.linear.x = <Giá_trị_float_tốc_độ_trục_X_do_AI_tính_toán>
msg.twist.linear.y = <Giá_trị_float_tốc_độ_trục_Y>
msg.twist.linear.z = <Giá_trị_float_tốc_độ_trục_Z>

# 3. Publish msg vào topic /servo_node_main/delta_twist_cmds ở mỗi khung hình (frame)
publisher_.publish(msg)
```

**Lưu ý kỹ thuật Git:**
Mọi code hiện đang nằm trên nhánh (Branch) `Robot_Controller_Servo`. Để đưa lên Github, hãy chạy lệnh: `git push -u origin Robot_Controller_Servo`. Bạn có thể thoải mái push các file code Python AI của mình vào nhánh này.
