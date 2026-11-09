# Phân tích trước khi thay đổi

Đã đọc `README.md`, `src/zlac_node.cpp`, `include/zlac_driver/zlac_driver.hpp`,
`scripts/check_velocity.py`; đối chiếu thêm implementation driver/encoder.
Đây là mô tả **bản gốc**, trước khi bổ sung web và watchdog.

## Luồng lệnh

`geometry_msgs/Twist /cmd_vel` → callback `ZlacDriverNode` → `send_velocity` →
động học vi sai → RPM → nhân chiều lắp bánh → cắt RPM từng bánh → `std::lround`
→ hai word int16 tại `0x2088` (trái), `0x2089` (phải), Modbus FC 0x10.
Driver mở RS485 115200, 8N1, slave 1. Một tiến trình gồm hai node, executor
đơn luồng đảm bảo giao dịch đọc/ghi không chồng nhau.

- `linear.x = v`, m/s, dương tiến; `angular.z = ω`, rad/s, dương quay trái.
- `vL = v − ωL/2`, `vR = v + ωL/2`.
- `RPM = v_bánh / r × 60 / (2π)`.
- `L=0,40 m`, `r=0,0825 m`, `rpm_max=300`, `max_linear_speed=3 m/s`.
- Tốc độ tiếp tuyến tối đa mỗi bánh: `300 × 2π/60 × 0,0825 ≈ 2,591814 m/s`.
- Tốc độ quay tại chỗ tối đa suy ra: `2 × 2,591814 / 0,40 ≈ 12,95907 rad/s`.
  Đây là giới hạn toán học, **không phải tốc độ thử an toàn được khuyến nghị**.
- Dấu phần cứng `dir_left=+1`, `dir_right=−1`: đi tiến có RPM thô trái dương,
  phải âm. Encoder đã bù cùng quy ước; không đảo lại trong web.
- `|v| > 3` ở bản gốc bị bỏ qua và giữ lệnh cũ; chưa chặn NaN/Infinity hoặc ω.
- Cắt RPM riêng từng bánh thay đổi tỉ lệ bánh, do đó có thể đổi độ cong quỹ đạo.

## Encoder và đối chiếu

Timer 20 ms đọc FC 0x03, 6 thanh ghi từ `0x20A7`. Vị trí thô là int32 counts;
tốc độ là int16 theo 0,1 RPM. `JointState /encoder` có hai joint
`left_wheel_joint`, `right_wheel_joint`, `position` rad cộng dồn từ 0,
`velocity` rad/s, đã nhân dấu lắp bánh. Timestamp là thời điểm nhận phản hồi.
Counts mỗi vòng = số vạch đọc từ phần cứng × 4; hệ số này chưa được kiểm chứng
độc lập trên phần cứng. Dữ liệu tốc độ không phụ thuộc hệ số counts này.

`check_velocity.py` tính tốc độ dự kiến bằng động học → cắt ±rpm_max →
làm tròn nửa ra xa 0 (không dùng Python round) → nhân `2π/60 × r`.
Ví dụ `v=0,05`, `ω=0` → 5,787 RPM → 6 RPM → 0,051836 m/s.
Đo = `JointState.velocity × r`; sai lệch = đo − dự kiến, m/s.
Tốc độ dự kiến là mục tiêu sau lượng tử hóa, chưa bao gồm độ trễ tăng/giảm tốc.

## Rủi ro phát hiện

1. Không watchdog; ngừng nguồn `/cmd_vel` không dừng động cơ.
2. Node bỏ qua kết quả `send_velocity`; lệnh không hợp lệ có thể giữ lệnh cũ.
3. ACK ghi chưa kiểm đầy đủ CRC, chiều dài, slave, function và register.
4. Read serial có timeout mỗi lượt khoảng 1 s; đọc từng phần có thể kéo dài,
   gây chậm các callback cùng executor. `tcdrain` không có deadline.
5. Enable ngay khi khởi tạo; chưa ghi 0 trước enable, và lỗi khởi tạo giữa chừng
   chưa được xử lý dừng/disable đầy đủ.
6. Tắt hoặc lỗi encoder vẫn có thể tiếp tục điều khiển; dữ liệu cũ dễ bị hiểu nhầm.
7. Nhiều publisher `/cmd_vel` có thể tranh nhau. `Twist` không chứa timestamp.
8. Watchdog phần mềm không cứu được khi chính driver process/OS chết hoặc bus
   không thể truyền lệnh. Stop là yêu cầu 0, không chứng minh đứng yên hay cắt mô-men.

## Thiết kế cuối theo yêu cầu giữ nguyên driver/encoder

Package `../zlac_driver_encoder` được giữ nguyên từng file so với bản gốc.
Không sửa CRC, serial timeout, startup enable, callback, CMake hoặc README của package đó.

Web, cầu nối HTTP/ROS và node giám sát nằm trong `zlac_motor_web`:

`trình duyệt → app.py → /cmd_vel → guard.py → topic riêng → driver gốc`

`driver/encoder gốc → /encoder → app.py → trình duyệt`

Guard chạy watchdog monotonic và kiểm đầu vào trước khi chuyển tiếp. Khi chạy,
guard tự khởi động driver gốc với remap `/cmd_vel` sang topic riêng. Helper build
đọc cấu hình từ header nguồn và xây bản gốc vào thư mục `.runtime` của web, sau đó
kiểm fingerprint nguồn/binary để cấu hình hiển thị khớp bản thực sự khởi động.
Guard không truy cập serial. Web cũng không truy cập serial.

Các hạn chế của bản gốc vẫn tồn tại: timeout đọc 1 s, ACK ghi chưa kiểm đầy đủ,
enable trước lệnh zero đầu tiên, không có watchdog nội bộ. Guard chết thì Linux
parent-death signal yêu cầu driver thoát mềm (SIGINT); đây là best effort, không
thay thế E-stop phần cứng. Không tuyên bố thời gian bánh dừng là 0,5 s.
