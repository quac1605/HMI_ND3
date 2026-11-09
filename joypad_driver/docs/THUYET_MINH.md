# Thuyết minh chương trình driver giữa joypad và robot

Tài liệu mô tả mã nguồn hiện tại sau khi tách module ngày 09/11/2026, làm cơ sở
cho báo cáo công việc 3.6, gồm kiến trúc, thuật toán và kiểm chứng phần mềm.

## 1. Mục tiêu và phạm vi

Chuyển đầu vào điều khiển hai trục thành vận tốc tịnh tiến `v` và vận tốc góc
`ω` của robot vi sai. Người vận hành chọn bốn tốc độ riêng cho tiến, lùi, trái,
phải. Độ lệch joystick quy định tỷ lệ tốc độ, cho phép đi và quay đồng thời.
Giữ thao tác để chạy; thả hoặc có sự kiện gián đoạn thì yêu cầu vận tốc zero.

Đầu vào được triển khai: joystick ảo bằng chuột/cảm ứng qua Pointer Events,
WASD, hoặc giữ Space kết hợp mũi tên. Chưa triển khai tay cầm vật lý qua USB,
Bluetooth, Gamepad API, HID hoặc topic `/joy`. Nếu cần các thiết bị đó, phải bổ
sung adapter đọc thiết bị và kiểm thử dead-man/ngắt thiết bị riêng.

## 2. Phân chia thành phần

```text
Chuột / cảm ứng / bàn phím
           ↓ sự kiện
joypad_driver/src/joypad_driver.js
  kiểm tra thao tác → trục x,y → vùng chết/chuẩn hóa → (v,ω)
           ↓ callbacks + command()
zlac_motor_web/static/app.js
  hiển thị + arm/lease/sequence/ticket + gửi HTTP mỗi 50 ms khi được cấp quyền
           ↓ HTTP cùng origin
zlac_motor_web/app.py + web_core.py
  kiểm tra giới hạn + watchdog web → geometry_msgs/Twist /cmd_vel
           ↓ ROS
zlac_motor_web/guard.py
  watchdog ROS → topic riêng → driver ZLAC nguyên bản → RS485 → động cơ

Encoder nguyên bản → JointState /encoder → app.py → màn hình đối chiếu
```

`joypad_driver` không phụ thuộc cấu trúc HTML hoặc ID của giao diện, không gọi
HTTP/ROS/serial. Adapter web cung cấp phần tử, callbacks và cấu hình. Như vậy
có thể kiểm thử thuật toán riêng và dùng lại trong một giao diện khác.

`zlac_driver_encoder` giữ nguyên. Giới hạn RPM, làm tròn, dấu cơ khí và tính
tốc độ encoder vẫn nằm ở các lớp đã có; không nhân dấu bánh thêm trong joypad.
Chỉ có một bản source thuật toán, được server phục vụ trực tiếp từ thư mục mới.

## 3. Thuật toán chuẩn hóa đầu vào

### 3.1. Tọa độ và vùng chết

Với tâm joystick `(cx, cy)`, điểm chạm `(px, py)` và bán kính thao tác `R=71 px`:

```text
x0 = (px − cx) / R       # dương sang phải
y0 = −(py − cy) / R      # dương về trước; đảo trục y của màn hình
m  = sqrt(x0² + y0²)

nếu m < 0,05: (x,y) = (0,0)
nếu m ≥ 0,05: (x,y) = (x0,y0) / max(1,m)
```

Vùng chết là hình tròn 5%, giữ nguyên tỷ lệ ngoài vùng chết; không kéo giãn
phần còn lại về 0–1. Điểm vượt vòng ngoài bị chuẩn hóa về biên, giữ hướng.
Chuẩn hóa tránh tăng tổng độ lệch khi nhấn chéo hoặc kéo ngoài vùng thao tác.

### 3.2. Quy đổi tốc độ và dấu

Gọi bốn giá trị không âm là `V_tiến`, `V_lùi` (m/s), `Ω_trái`, `Ω_phải` (rad/s):

```text
v = y × (V_tiến nếu y ≥ 0, ngược lại V_lùi)
ω = −x × (Ω_trái nếu x ≤ 0, ngược lại Ω_phải)
```

Do đó `linear.x > 0` là tiến; `angular.z > 0` là quay trái. Bốn giá trị được
kiểm tra hữu hạn, không âm và không vượt giới hạn cấu hình nhận từ cầu nối.
Ô nhập giữ bước tăng/giảm **0,1**; vẫn có thể gõ thủ công tốc độ thử thấp hơn.

Ví dụ với `V_tiến=0,1`, `V_lùi=0,1`, `Ω_trái=Ω_phải=0,3`:

| Thao tác | x | y | v (m/s) | ω (rad/s) |
| --- | ---: | ---: | ---: | ---: |
| W / kéo lên tối đa | 0 | 1 | 0,100000 | 0 |
| S / kéo xuống tối đa | 0 | −1 | −0,100000 | 0 |
| A / kéo trái tối đa | −1 | 0 | 0 | 0,300000 |
| D / kéo phải tối đa | 1 | 0 | 0 | −0,300000 |
| W+A | −1/√2 | 1/√2 | 0,070711 | 0,212132 |
| Thả toàn bộ | 0 | 0 | 0 | 0 |

### 3.3. Bàn phím và nhiều nguồn đầu vào

WASD tạo `x0 = D − A`, `y0 = W − S`, trong đó phím đang giữ có giá trị 1.
Hai hướng đối nhau triệt tiêu. Thả một phím trong tổ hợp thì còn hướng của
phím đang giữ; thả hết WASD thì yêu cầu dừng. Space là phím giữ để chạy cho chế
độ mũi tên; thả Space thì dừng.

Mỗi lần giữ chỉ có một nguồn: pointer, WASD hoặc mũi tên. Pointer ID đầu tiên
giữ quyền điều khiển; ngón tay thứ hai không được lái hoặc kết thúc thao tác
của ngón tay đầu tiên. Pointer capture giữ theo dõi khi kéo ra ngoài vùng.

Không nhận điều khiển khi đang gõ input/textarea/select/contenteditable, dùng
Ctrl/Alt/Meta hoặc IME. Sử dụng `KeyboardEvent.code` để Shift/Caps Lock không
đổi ánh xạ. Sự kiện auto-repeat không được arm lại sau Dừng/timeout; người dùng
phải có lần nhấn mới. Escape luôn yêu cầu dừng.

## 4. Trạng thái và dừng

```text
IDLE ── nhấn mới + canStart + focus + tab hiện ──► HOLDING
HOLDING ── thay đổi vị trí/phím ──► cập nhật (x,y)
HOLDING ── thả/cancel/mất capture/Dừng/gián đoạn ──► IDLE + (0,0) + onStop
```

HOLDING chỉ là trạng thái thao tác. Adapter web còn phải chờ arm trả lease
hợp lệ mới gửi lệnh chuyển động. Lệnh dừng tăng `stopGeneration`; kết quả arm
đến muộn với thế hệ cũ không được kích hoạt lại. Sequence/ticket phía server
chặn lệnh cũ hoặc quá hạn. Không đổi các cơ chế này khi tách module.

Các lớp xử lý lỗi:

| Lớp | Sự kiện / điều kiện | Hành động |
| --- | --- | --- |
| Joypad | Thả pointer, cancel, mất capture, thả hết WASD/Space, Escape | Xóa phím/pointer, đưa trục về zero, gọi `onStop`. |
| Joypad | Mất focus, ẩn tab, pagehide, offline | Reset và yêu cầu adapter gửi stop beacon. |
| Adapter web | Đổi tốc độ, mất lease/ready, HTTP lỗi hoặc không phản hồi >350 ms | Gọi module stop và gửi yêu cầu dừng. |
| HTTP backend | Không nhận lệnh mới trong `--web-timeout` (mặc định 250 ms) | Thu hồi lease, phát Twist zero. |
| ROS guard | Không nhận `/cmd_vel` mới trong `--command-timeout` (mặc định 500 ms), encoder quá hạn/lệnh sai | Yêu cầu zero đến driver qua topic riêng. |

Nếu JavaScript bị treo, sự kiện frontend không chạy được; watchdog server và
ROS là các lớp độc lập. Stop beacon có thể mất trên mạng. Giới hạn thời gian
trên là thời điểm phần mềm yêu cầu dừng, không phải chứng nhận thời gian dừng
cơ khí. Driver gốc có thao tác serial chặn; lỗi điện/RS485 có thể ngăn lệnh đến
động cơ. Vẫn cần E-stop phần cứng độc lập.

## 5. Động học và phản hồi để đánh giá kết quả

Joypad tạo vận tốc thân xe; lớp robot hiện có xử lý tiếp:

```text
u_L = v − ωL/2                       u_R = v + ωL/2
rpm_i = u_i/r × 60/(2π)
rpm_gửi_i = lround(clamp(rpm_i, −RPM_max, RPM_max))
u_dự_kiến_i = rpm_gửi_i × 2π/60 × r
u_đo_i = JointState.velocity[tên_joint_i] × r
sai_lệch_i = u_đo_i − u_dự_kiến_i
```

`L` là khoảng cách bánh, `r` là bán kính bánh. `lround` làm tròn nửa ra xa 0,
khớp code gốc và `scripts/check_velocity.py`. Cấu hình hiện tại: `L=0,40 m`,
`r=0,0825 m`, `RPM_max=300`. Không hard-code các giá trị này trong joypad.

Phân biệt trong báo cáo: ý định từ joypad; lệnh đã phát lên ROS; lệnh guard
chuyển tiếp; số đo encoder mới. Đã phát/chuyển tiếp không có nghĩa động cơ đã
thực hiện. Encoder giả lập không được dùng làm kết quả thử động cơ thật.

## 6. Kiểm chứng và giới hạn

Các bộ kiểm thử liên quan:

| Bộ kiểm thử | Nội dung |
| --- | --- |
| `joypad_driver/tests/test_joypad_driver.js` | 12 ca độc lập: vùng chết, tỷ lệ/đường chéo, dấu, đầu vào sai, nhiều pointer, cancel/capture, focus, offline, tháo listener. |
| `zlac_motor_web/tests/test_keyboard.js` | 16 ca tích hợp module với adapter web: WASD, Space, ô nhập, Dừng, arm trễ, focus, timeout, đổi tốc độ và gửi stop từ pointer. |
| `zlac_motor_web/tests/test_web_core.py` | 10 ca kiểm tra động học, encoder, quyền điều khiển và watchdog web. |
| `zlac_motor_web/tests/test_http.py` | 4 ca demo HTTP: điều khiển/dừng/mất encoder, Host/Origin, nạp đúng source module, lỗi khi thiếu source. |
| `zlac_motor_web/tests/test_ros_integration.py` | Driver C++ nguyên bản + guard + bridge trên ROS domain 176 và Modbus PTY giả lập. |

Kết quả sau lần tách module: **43 ca đạt** (12 module + 16 frontend + 10 core +
4 HTTP + 1 ROS/PTY). Bộ tích hợp ROS/PTY hoàn thành trong khoảng 29 giây.
Checksum mã nguồn driver/encoder khớp trước và sau thay đổi.

Lệnh chạy nằm trong [README](../README.md#kiểm-thử). Các kiểm thử JavaScript
dùng DOM giả nên không thay thế việc thao tác trên thiết bị chuột/cảm ứng thật.
Kiểm thử ROS/PTY không dùng cổng RS485 thật. Chưa xác minh phần cứng sau lần
tách này: chiều lắp bánh, dấu/độ chính xác encoder, thời gian dừng thực tế,
watchdog firmware và E-stop. Làm theo quy trình kê bánh trong README của web.

Để hoàn thiện báo cáo thực nghiệm, bổ sung sau khi đo: cấu hình xe, phiên bản
mã nguồn, bảng lệnh/dự kiến/encoder, thời điểm và tuổi dữ liệu, độ trễ dừng đo
được, ảnh thử bánh kê và kết quả từng tình huống mất kết nối. Không suy diễn
các số liệu đó từ test giả lập.
