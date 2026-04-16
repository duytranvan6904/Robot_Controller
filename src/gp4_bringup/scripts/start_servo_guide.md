# Hướng dẫn chạy và sử dụng Node Cartesian Streamer cho tay máy Yaskawa GP4

Tài liệu này ghi lại chi tiết các bước cần thực hiện để khởi động robot và chạy script `cartesian_streamer.py` nhằm điều khiển chuyển động Cartesian liên tục ở tần số cao (thuộc chuẩn streaming).

## 1. Các bước khởi động

Bạn cần mở 2 Terminal riêng biệt. **Lưu ý kiến trúc quan trọng: KHÔNG chạy file `real_robot.launch.py`**. File đó chứa config bật `ros2_control` sẽ xảy ra xung đột điều khiển phần cứng của API `MotoROS2`.

### Terminal 1: Khởi động hệ thống cơ bản
Chạy MoveIt (`move_group`), ROS 2 Robot State Publisher, restamp joint states và hiển thị mô hình trong giao diện RViz.

```bash
source /home/duy/Downloads/gp4_ws/install/setup.bash
ros2 launch gp4_moveit_config gp4_start.launch.py
```

### Terminal 2: Chạy Node tích hợp giải IK và Streamer
Sau khi Terminal 1 khởi chạy xong, dùng Python script để nối với IK Solver và driver MotoROS2.
Nếu chạy với thực tế, hãy chắc chắn Docker micro-ROS agent đã bật từ trước.

```bash
source /home/duy/Downloads/gp4_ws/install/setup.bash
cd /home/duy/Downloads/gp4_ws/src/gp4_bringup/scripts
python3 cartesian_streamer.py
```

Lệnh trên không truyển cờ gì, cho nên script chạy dưới dạng lắng nghe AI / Camera Node, đợi tọa độ mới rơi vào topic.

---

## 2. Các tham số dòng lệnh `--demo` (Dùng để chạy thử/Test mượt)

Trong trạng thái không có AI, bạn dùng argument `--demo <pattern>` để bắt robot liên tiếp vẽ ra quỹ đạo mẫu tự sinh ở script. 

- **Chạy theo đường thẳng (Line)**: Robot tịnh tiến qua lại dọc theo trục X (biên độ 5cm).
  ```bash
  python3 cartesian_streamer.py --demo line
  ```

- **Chạy theo hình tròn (Circle)**: Robot vẽ một hình tròn trên mặt phẳng YZ (bán kính 5cm).
  ```bash
  python3 cartesian_streamer.py --demo circle
  ```

- **Chạy vòng Lissajous**: Robot vẽ đường cong hình số 8 / vô cực trên mặt phẳng XY.
  ```bash
  python3 cartesian_streamer.py --demo lissajous
  ```

---

## 3. Ý nghĩa thông số Code tự tuỳ biến (Tune Robot)

Nằm ở những dòng code đầu của file `cartesian_streamer.py`, bạn có thể can thiệp vào các hằng số nhằm khắc phục khi robot giật hoặc chậm:

| Thông số ở trong Code | Chỉ số ví dụ | Chức năng & Tác động |
|-----------------|--------------|----------------------|
| `WS_X`, `WS_Y`, `WS_Z` | `(-0.7, 0.7)` | **Vùng giới hạn không gian (Workspace)** giới hạn bằng hệ mét (m). Nơi định danh robot an toàn hoạt động. Nếu AI điều phối tọa độ rời ra ngoài, tọa độ này sẽ bị reject bằng cảnh báo ở log. |
| `POINT_DURATION_SEC`| `1.0 / 30.0` | **Tần số/Thời gian cấp một điểm tới driver (Rate tick)**. Thông số này đại diện yêu cầu thời gian hoàn thành một vi bước nhỏ. Đang mặc định 30Hz, nếu robot Yaskawa vật lý thấy chập chờn hoặc không tải nổi, khuyên đổi thành `1.0 / 22.0` hoặc `1.0 / 20.0`. |
| `IK_TIMEOUT_SEC` | `0.2` (giây)| **Thời gian IK rớt mạng**. Giới hạn để bộ tính MoveIt IK Solver ngưng việc cắm CPU tìm góc xoay cho toạ độ. Nếu chậm thì nên tăng, nhưng tăng thì có cơ may kéo hụt luôn chu kì stream 30Hz. |
| `SMOOTH_ALPHA`| `0.5` | **Hệ số vuốt quỹ đạo / Làm mượt nội suy** `(0.0 -> 1.0)`. Giúp chống nhảy khớp giật cục khi AI gửi vị trí mới ở xa. Nếu robot chuyển động quá chậm, hãy tăng `0.7 - 0.8` để bám sát và nhạy với mục tiêu thực hơn. |
| `MAX_JOINT_DELTA`| `0.5` (rad) | **Bảo vệ gia tốc khớp**. Dù vị trí AI hay IK đưa đúng, góc quay khớp (joint angle) không được vượt quá chỉ số này so với tư thế tíc tắc vừa rồi để tranh hư Servo ở tay máy. |

---

## 4. Tương tác với AI Node hoặc Camera ngoại vi

Quá trình điều khiển chuẩn không dùng `--demo` thì sẽ có các Data Topic đi và về trong ROS 2 như sau:

- **Nhận (Subscribe)**: 
  - Toạ độ điểm cuối + Xoay góc (Position + Orientation) ở `/cartesian_streamer/target_pose` (chuẩn `geometry_msgs/PoseStamped`).
  - Lối tắt tiện dụng: Chỉ cần 3 trục xyz `[x, y, z]`, góc xoay tự khóa y thói cũ ở `/cartesian_streamer/target_xyz` (chuẩn `std_msgs/Float64MultiArray`).
- **Gửi (Publish)**: Thông báo lại EE pose thực tại của robot quay về cho AI theo dõii tiến trình hằng phần ở `/cartesian_streamer/current_pose`.
