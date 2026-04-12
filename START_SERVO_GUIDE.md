# Hướng Dẫn Điều Khiển Quỹ Đạo Liên Tục GP4 Trong Thực Tế

Tài liệu này hướng dẫn chi tiết cách chạy hệ thống điều khiển liên tục (streaming tọa độ liên tiếp) cho tay máy Yaskawa GP4 **trên phần cứng thật**. Hệ thống này đặc biệt thiết kế để tích hợp với Camera AI, ML Inference hoặc thiết bị Teleoperation cầm tay.

---

## 💡 Lưu ý quan trọng: Khác biệt giữa RViz Mô phỏng và Thực tế
Trong môi trường mô phỏng (`sim.launch.py`), do sử dụng *fake controller*, trạng thái `/joint_states` không tự động cập nhật lại thời gian thực sau khi nội suy xong. Bạn sẽ thấy hiện tượng robot phải "dừng lại và reset" mới plan được điểm tiếp theo.

**Tuy nhiên trên phần cứng thật:** Trạng thái của các khớp động cơ (Encoder) được gửi liên tục về ROS 2 ở tần số cao. Điều này giúp hệ thống liên tục lấy được vị trí vật lý tuyệt đối của robot và mượt mà "tiếp nối" các quỹ đạo với nhau mà **không bao giờ bị khựng hay yêu cầu reset**.

---

## 🛠 QUY TRÌNH CHẠY TRÊN PART CỨNG THẬT 

### 1. Khởi động phần cứng (Terminal 1)
Bạn dọn dẹp các tiến trình cũ và khởi chạy trực tiếp kết nối với tủ điện Yaskawa YRC1000micro thay vì dùng mô phỏng.

```bash
cd ~/Downloads/gp4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Lệnh khởi động real hardware (cần sửa lại YOUR_ROBOT_IP nếu IP không mặc định)
ros2 launch gp4_bringup hw.launch.py robot_ip:=192.168.1.33
```
*Đảm bảo bạn nhìn thấy `hw_adapter_node` báo Controller đã `Active` và MotoROS2 đã báo `Connected`.*


### 2. Hai Phương Pháp Điều Khiển Liên Tục

Trong thực tế hệ thống hiện tại của chúng ta, có **Hai cách (Option)** để thực thi chuỗi lệnh liên tục. Tùy thuộc vào yêu cầu của thuật toán AI mà bạn chọn cách phù hợp:

#### Option A: Điều khiển thông qua `MOVE_REL` (Khuyên dùng & An toàn nhất)
**Đặc điểm:** Đi qua đầy đủ kiến trúc bảo mật của hệ thống `AI -> Safety -> Motion Core -> Hardware`. Cực kỳ an toàn vì hệ thống sẽ kiểm tra va chạm (Collision scene) và tự động rà mượt gia tốc Ruckig trước khi gửi.

1. **Khởi chạy Controller Client (Terminal 2):**
```bash
cd ~/Downloads/gp4_ws
source install/setup.bash
python3 src/gp4_bringup/test_continuous_move.py
```

2. **Áp dụng cho Code AI của bạn:** 
Bạn gọi action client `/execute_motion` tương tự file thử nghiệm. Liên tục bắn các bước `MOVE_REL` (delta_x, delta_y, delta_z). 
Vì kiến trúc của chúng tôi hỗ trợ buffering chuẩn xác, các mục tiêu sẽ được nối đuôi nhau di chuyển tay máy liên tiếp.

#### Option B: Điều khiển trần (Raw) bằng MoveIt Servo 30Hz
**Đặc điểm:** Bypass màng lọc an toàn `motion_core`, giao tiếp trực tiếp với bộ điều khiển để đạt tốc độ nội suy phản hồi **độ trễ bằng 0** (chuẩn 30Hz Teleoperation). 
*Chỉ dùng khi bạn tin tưởng hoàn toàn vào dữ liệu do AI xuất ra không bị nhiễu làm gãy trục.*

1. Trong `hw.launch.py`, bảo đảm node `servo_node` đã được thêm vào luồng Launch.
2. Bạn cần đẩy API Twist liên tục (ví dụ Python cho luồng AI):
```python
from geometry_msgs.msg import TwistStamped
import rclpy

# ... (Khởi tạo node rclpy)
publisher = node.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

msg = TwistStamped()
msg.header.frame_id = 'base_link'

# Liên tục nhồi vận tốc mong muốn (m/s) vào trục X, Y, Z (Tần suất 30 fps từ Camera)
msg.twist.linear.x = 0.05  # Tiến 5cm/s
msg.twist.linear.y = 0.0
msg.twist.linear.z = 0.0

publisher.publish(msg)
```

---

## 🚦 Những Kiểm Tra An Toàn (Safety Checks) Khi Chạy Thật
Vì đang gửi tọa độ liên tục, hãy tuân thủ 3 nguyên tắc sống còn khi làm việc với robot công nghiệp:

1. **Lệnh STOP khẩn cấp (E-Stop):**
Luôn có 1 tay cầm bộ Teach Pendant của Yaskawa. Khi robot có xu hướng di chuyển bất thường, **nhấn ngay nút E-Stop màu đỏ**.
2. **Setup Vận Tốc Nhỏ Ban Đầu:**
Tại file script AI, hãy để hệ số Vận tốc ban đầu ở mức ~`0.10` (10% tốc độ thực) và Biên độ thay đổi `Tối đa 1 cm/bước` để test hướng đi của robot.
3. **Quan sát thông điệp Cảnh báo (Warn):**
Nhìn vào Terminal 1, nếu `hw_adapter_node` báo dòng màu vàng `Tolerance exceeded` hay `Delay...`, tức là vòng lặp xuất tọa độ tốc độ 30Hz của Camera đang bị trễ so với chu kỳ vật lý, bạn cần tối ưu hóa code inference của AI nhẹ lại để bù khung hình hình ảnh.
