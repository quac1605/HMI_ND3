# ZLAC8015D Driver — Bộ chấp hành (v, ω) + encoder cho xe AMR

Thư viện C++ tối giản điều khiển driver động cơ **ZLAC8015D** (điều khiển 2
bánh vi sai) qua **RS485 / Modbus RTU**. Vai trò của lớp này trong hệ thống:
nhận lệnh vận tốc `(v, ω)` từ tầng trên và biến thành chuyển động bánh xe,
đồng thời đọc encoder hai bánh và phát lên topic `/encoder`.
Nó **không** làm định vị và **không** tính odom — trong kiến trúc dự kiến, SLAM
(IMU + LiDAR + camera) lo việc đó; dữ liệu `/encoder` do khối khác lấy về xử lý.

Package ROS 2 (`ament_cmake`) gồm thư viện lõi `zlac_driver_lib` (thuần C++:
`ZlacDriver` điều khiển bánh + `ZlacEncoder` đọc encoder) và executable `zlac_node`
— **một tiến trình chứa hai node**: `zlac_driver` (sub `/cmd_vel` → `send_velocity`)
và `zlac_encoder` (đọc encoder 50 Hz → pub `/encoder`).

```
  Tầng trên (quyết định v, ω)
  =>  /cmd_vel  =>  node zlac_driver  =>  ZlacDriver (kinematics + Modbus ghi)
  =>  RS485 / Modbus RTU  =>  ZLAC8015D (driver HW) - tự đóng vòng tốc độ
  =>  [ bánh trái ]  [ bánh phải ]

  ZLAC8015D (bộ đếm encoder)  =>  ZlacEncoder (Modbus đọc)  =>  node zlac_encoder
  =>  /encoder  =>  khối khác (odom, SLAM...)
```


## 1. Nền tảng lý thuyết

### 1.1. Động học xe vi sai (differential drive)

Xe hai bánh chủ động cùng trục. Trạng thái chuyển động mô tả bằng hai đại lượng:

- `v` — vận tốc dài của tâm xe (m/s), dương = tiến.
- `ω` — vận tốc góc quay của thân xe (rad/s), dương = ngược chiều kim đồng hồ
  (CCW) = rẽ trái. Đây là quy ước chuẩn ROS.

Hai tham số hình học:

- `L` (wheel_base) — khoảng cách giữa hai bánh (m).
- `r` (wheel_radius) — bán kính bánh (m).

**Inverse kinematics** — từ `(v, ω)` suy ra vận tốc từng bánh. Tâm xe đi với
`v`; bánh trái cách tâm `L/2` về một phía, bánh phải phía kia, nên khi xe quay
với `ω`, hai bánh có thêm thành phần vận tốc ngược dấu:

v_trái  = v − ω·(L/2)
v_phải  = v + ω·(L/2)

Đổi vận tốc dài của bánh (m/s) sang vận tốc góc bánh (rad/s) rồi sang RPM:

ω_bánh = v_bánh / r            (rad/s)
rpm    = ω_bánh · 60 / (2π)    (vòng/phút)


Trực giác kiểm chứng:
- `ω = 0`: hai bánh cùng RPM → đi thẳng.
- `v = 0, ω ≠ 0`: hai bánh RPM bằng nhau nhưng **ngược dấu** → xoay tại chỗ.

### 1.2. RS485 và Modbus RTU

**RS485** là chuẩn tín hiệu điện (lớp vật lý): truyền vi sai trên cặp dây A/B,
**half-duplex** — hai chiều dùng chung một đường, chỉ một bên nói tại một thời
điểm. Hệ quả quan trọng cho code: sau khi phát xong phải *đợi dây rảnh* rồi mới
chuyển sang nghe, nếu không sẽ tự nghe lại mình hoặc bỏ lỡ câu trả lời.

**Modbus RTU** là giao thức bên trên, mô hình **master–slave hỏi–đáp**:
- Master (máy tính) gửi frame yêu cầu.
- Slave (driver, địa chỉ `0x01`) *phải* trả lời — slave không tự phát.
- Mỗi lệnh là một chu trình: **gửi → chờ trả lời (ACK)**.

Cấu trúc một frame (ADU): `[địa chỉ][mã hàm][dữ liệu...][CRC 2 byte]`.
CRC (kiểm tra dư vòng) bảo vệ toàn vẹn — sai một bit là CRC không khớp.

Ba mã hàm dùng ở đây:
- **FC 0x06** — ghi 1 thanh ghi (dùng cho mode / enable / disable). Slave trả
  lời bằng cách **echo lại nguyên frame**.
- **FC 0x10** — ghi nhiều thanh ghi liên tiếp (dùng ghi tốc độ 2 bánh trong 1
  frame). Slave trả lời frame ngắn: địa chỉ + mã hàm + thanh ghi bắt đầu + số
  word đã ghi + CRC.
- **FC 0x03** — đọc thanh ghi (dùng đọc số vạch encoder, vị trí và tốc độ thực
  của hai bánh để phát `/encoder`). Slave trả lời: địa chỉ + mã hàm + số byte +
  dữ liệu + CRC. Riêng đường đọc này có kiểm CRC và độ dài frame.

**ACK** = frame trả lời của slave. Có ACK = driver còn sống, đã nhận lệnh.
Không có ACK (im lặng) = đứt kết nối ở đâu đó (sai địa chỉ, sai baud, đảo dây
A/B, mất nguồn, tuột dây). Vì `write()` chỉ đẩy byte ra cổng chứ không đảm bảo
có ai nhận, **chờ ACK là cách duy nhất biết lệnh có tới nơi**.

**Exception frame**: nếu slave *từ chối* lệnh (giá trị sai, sai mode...), nó
trả về frame với mã hàm bị bật bit cao nhất (0x06 → 0x86, 0x10 → 0x90), byte
kế là mã lỗi. Khác hẳn "im lặng" — đây là "nghe thấy nhưng không làm".

### 1.3. Thông số phần cứng (đã chốt qua bring-up test)


Cổng serial         `/dev/ttyUSB_driver` (udev symlink) 
Địa chỉ slave       `0x01` 
Baud / khung        115200, 8N1 
wheel_base `L`      0.40 m 
wheel_radius `r`    0.0825 m 
RPM_MAX             300 
Thanh ghi mode      `0x200D` (= 3: velocity mode) 
Thanh ghi enable    `0x200E` (8 = enable, 7 = disable) 
Thanh ghi tốc độ    `0x2088` = bánh TRÁI, `0x2089` = bánh PHẢI 

### 1.4. Quy ước dấu — điểm dễ nhầm nhất

Xác định bằng thực nghiệm (test từng bánh, rồi cả hai). Hai bánh lắp đối xứng
gương nên chiều quay thô ngược nhau: gửi +RPM vào `0x2088` làm bánh trái tiến,
còn +RPM vào `0x2089` làm bánh phải lùi. Để `(v, ω)` cho ra chuyển động đúng,
phải đảo dấu bánh phải:

DIR_LEFT  = +1.0
DIR_RIGHT = −1.0

(Ghi chú: ở bring-up ban đầu giá trị là `−1.0 / +1.0`. Khi thử trên xe thật với
máy nhúng, chạy chương trình ví dụ `zlac_example` (nay đã bỏ khỏi package) thì cả
hai bánh quay lùi, nên đã đảo cả hai dấu. Hướng lắp động cơ/dây pha có thể khác
giữa các xe — nếu gặp lại hiện tượng này, đảo dấu `dir_left`/`dir_right` trong
`ZlacConfig`. Node encoder lấy dấu từ đây, không phải sửa chỗ khác.)

Hệ quả phản trực giác, đáng ghi nhớ khi soi frame hex:
- **Đi thẳng** → hai word tốc độ ngược dấu nhau.
- **Xoay tại chỗ** → hai word cùng dấu.

(Vì phép đảo bánh trái và phép "xoay cần hai bánh ngược chiều" cộng lại làm
dấu tráo so với trực giác.)


## 2. Giải thích các hàm quan trọng

Chỉ nêu input → xử lý → output. Chi tiết cài đặt xem trong `zlac_driver.cpp`.

### `inverse_kinematics(v, omega, rpm_left, rpm_right)`
- **Vào:** `v` (m/s), `omega` (rad/s).
- **Ra:** `rpm_left`, `rpm_right` — RPM mong muốn của mỗi bánh (qua tham
  chiếu), **chưa** đảo dấu, **chưa** giới hạn.
- **Xử lý:** đúng công thức mục 1.1. Thuần toán học, không đụng phần cứng.
  Public để tầng trên có thể xem trước / log giá trị mà không cần gửi.

### `send_velocity(v, omega, clamped)` — hàm core
- **Vào:** `v`, `omega`.
- **Ra:** trả `bool` (false nếu gửi thất bại / mất kết nối); `clamped` qua
  tham chiếu = true nếu lệnh vượt RPM_MAX và đã bị cắt.
- **Xử lý:** 4 bước.
  1. `inverse_kinematics` → rpm mỗi bánh.
  2. Nhân dấu (`DIR_LEFT`/`DIR_RIGHT`) — bù chiều lắp ngược của bánh.
  3. Clamp về `±RPM_MAX`, làm tròn sang số nguyên 16-bit. Nếu bị cắt, bật cờ
     `clamped`.
  4. Đóng frame FC 0x10 và gửi (qua `send_double`).
- **Lưu ý về `clamped`:** khi bị clamp, không chỉ tốc độ sai mà **hướng cong
  cũng sai** — vì một bánh bị cắt còn bánh kia chưa, phá vỡ tỉ lệ trái/phải.
  Tầng trên nên xử lý cờ này (giảm lệnh, hoặc cảnh báo planner).

### `write_and_ack(frame, len, expected_len)` — lớp giao tiếp
- **Vào:** frame Modbus đã đóng hoàn chỉnh (gồm CRC), độ dài, và độ dài frame
  trả lời mong đợi (8 cho FC 0x06 và 0x10 ở velocity mode).
- **Ra:** `bool` — true nếu gửi trọn vẹn và có ACK hợp lệ.
- **Xử lý (mức "vừa đủ" cho AMR gửi liên tục 20–50 Hz):**
  1. `tcflush(TCIFLUSH)` — bỏ byte thừa còn trong buffer nhận trước khi gửi. Buffer
     nhận là của cổng, dùng chung với fd của `ZlacEncoder`: một câu trả lời đến trễ
     (sau khi bên hỏi đã timeout) sẽ nằm lại đó và bị đọc nhầm làm ACK, lệch frame
     từ đó về sau.
  2. `write` cả frame; sai số byte → lỗi (thường là tuột cổng).
  3. `tcdrain` — đợi phát xong hết mới nghe (bắt buộc với half-duplex).
  4. `read_exact` đọc đủ `expected_len` byte. Ngoài việc đọc đủ, thao tác này
     **hút sạch frame trả lời khỏi buffer**, tránh phần thừa lem sang lần gửi
     sau — cái bẫy làm chương trình chạy vài phút rồi loạn ACK ở tần số cao.
  5. 0 byte → timeout → mất kết nối → false.
  6. Byte mã hàm có bit 0x80 → exception (driver từ chối) → false.
  7. Còn lại → true.
- **Cố ý KHÔNG kiểm** CRC / đúng độ dài / đúng slave-fc ở tầng chạy: nhiễu lẻ
  tự khỏi ở frame kế tiếp (20 ms sau), không đáng công. Nếu bus nhiễu nặng
  thật thì mới cần thêm kiểm tra CRC / độ dài / slave-fc.

### `read_exact(fd, buf, want)` (nội bộ)
- **Vào:** file descriptor, buffer, số byte cần.
- **Ra:** số byte thực đọc được (< want nếu timeout giữa chừng).
- **Vì sao cần:** một `read()` có thể trả về ít hơn yêu cầu vì dữ liệu RS485
  tới thành cụm rời rạc. Phải lặp đọc đến khi đủ hoặc timeout.

### Vòng đời: `open / set_velocity_mode / enable / disable / stop / close`
Trình tự chuẩn: `open → set_velocity_mode → enable → (send_velocity lặp) →
stop → disable → close`. `stop()` gửi tốc độ 0 mà **không** disable (dừng mềm);
`disable()` mới cắt mô-men. Mỗi hàm trả `bool`, kiểm ở tầng trên.

**Khóa cổng.** `open()` mở cổng rồi **khóa độc quyền** nó (`flock(LOCK_EX | LOCK_NB)`),
giữ đến `close()`. Chương trình thứ hai mở cổng (vd `zlac_node` thứ hai do người khác
ssh vào chạy) bị từ chối ngay trong `open()`, trước khi kịp gửi frame nào, nên không
ảnh hưởng bản đang chạy. Kernel tự nhả khóa khi đóng fd hoặc tiến trình thoát — kể cả
crash hay `kill -9` — nên không có chuyện khóa bị kẹt. Đây là khóa "tự nguyện": chương
trình không dùng `flock` (minicom, script Python...) vẫn mở và ghi cổng được.

### `ZlacEncoder` — đọc encoder (`zlac_encoder.hpp/.cpp`)
Class riêng, không phụ thuộc ROS và không phụ thuộc `ZlacDriver`. Tự mở một fd
riêng tới cùng cổng `/dev/ttyUSB_driver` (cấu hình termios y hệt `ZlacDriver::open()`).
Chỉ **đọc** (FC 0x03), không bao giờ ghi. **Không khóa cổng:** fd của nó nằm cùng
tiến trình với `ZlacDriver`, nếu cũng xin khóa độc quyền sẽ bị chính khóa của
`ZlacDriver` chặn.

- `read_encoder_lines(left, right)` — đọc số vạch encoder ở `0x2030` (trái),
  `0x2060` (phải). Gọi một lần lúc khởi động.
- `read_feedback(fb)` — một frame FC 0x03 đọc 6 thanh ghi liền nhau từ `0x20A7`
  (vị trí + tốc độ hai bánh, xem mục 4.2), nên hai bánh được lấy mẫu cùng lúc.
  Trả dữ liệu **thô**, chưa nhân dấu `dir_left`/`dir_right`.
- `read_registers(reg, count, out)` (nội bộ) — khác `write_and_ack`, đường đọc
  này **có** kiểm đủ độ dài, đúng slave/mã hàm, số byte và CRC, vì dữ liệu đọc về
  được đem đi tính toán. Trước khi gửi còn `tcflush` bỏ byte thừa của lần trước.

---

## 3. Cách build và chạy

### 3.1. Cấu trúc package
```
zlac_driver/
├── package.xml                      ← ament_cmake, deps: rclcpp, geometry_msgs, sensor_msgs
├── CMakeLists.txt                   ← thư viện zlac_driver_lib + executable zlac_node
├── include/zlac_driver/
│   ├── zlac_driver.hpp              ← ZlacConfig + ZlacDriver: điều khiển bánh (không phụ thuộc ROS)
│   └── zlac_encoder.hpp             ← ZlacEncoderConfig + ZlacEncoder: đọc encoder (không phụ thuộc ROS)
├── src/
│   ├── zlac_driver.cpp              ← cài đặt ZlacDriver
│   ├── zlac_encoder.cpp             ← cài đặt ZlacEncoder
│   └── zlac_node.cpp                ← 2 node ROS 2 (zlac_driver + zlac_encoder), chung 1 executor
└── scripts/
    └── check_velocity.py            ← công cụ test: so vận tốc lệnh với encoder (mục 4.4), không cài qua colcon
```
Executable cài ra: `zlac_node` (một tiến trình, hai node).

### 3.2. Build
Đặt package trong `src/` của một workspace colcon (hoặc symlink vào):
```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select zlac_driver
source install/setup.bash
```
Đã build thử thành công trên ROS 2 Jazzy (không warning).

### 3.3. Chuẩn bị trước khi chạy
- **Kê chân đế cho cả hai bánh khỏi mặt đất** ở mọi lần chạy thử.
- Kiểm quyền cổng: `ls -l /dev/ttyUSB_driver`; nếu bị từ chối,
  `sudo usermod -aG dialout $USER` rồi đăng xuất/đăng nhập lại.
- Nếu symlink `/dev/ttyUSB_driver` chưa có, kiểm `ls /dev/ttyUSB*` và sửa
  `device` trong `ZlacConfig` (thường `/dev/ttyUSB0`).

### 3.4. Chạy
```bash
# Cả hai node: điều khiển bánh (nghe /cmd_vel) + phát /encoder
ros2 run zlac_driver zlac_node
# Chỉ điều khiển bánh, không đọc encoder:
ros2 run zlac_driver zlac_node --ros-args -p zlac_encoder:enabled:=false
```
Bật/tắt encoder trong lúc đang chạy: xem tham số `enabled` ở mục 4.2.

Ctrl+C bất cứ lúc nào: chương trình gửi tốc độ 0 + disable trước khi thoát (Ctrl+C
tắt cả hai node vì chúng chung một tiến trình).

### 3.5. Đổi thông số
Node hiện dùng `ZlacConfig` mặc định trong `zlac_driver.hpp` (không đọc tham số ROS,
trừ tham số `enabled` của node encoder — mục 4.2).
Muốn đổi `device`, `wheel_base`, `rpm_max`... thì sửa giá trị mặc định trong
`include/zlac_driver/zlac_driver.hpp` rồi build lại. Node encoder lấy `device`, `slave`,
`dir_left`, `dir_right` từ chính `ZlacConfig` này (gán trong `main` của `zlac_node.cpp`),
nên chỉ sửa một chỗ; riêng `counts_per_line` nằm trong `ZlacEncoderConfig`
(`zlac_encoder.hpp`). Khi dùng thư viện trực tiếp thì đặt qua `ZlacConfig`:
```cpp
ZlacConfig cfg;
cfg.device      = "/dev/ttyUSB0";
cfg.wheel_base  = 0.42;
cfg.rpm_max     = 200;
ZlacDriver drv(cfg);
```

### 3.6. Giới hạn cần biết (an toàn)
- **Không có watchdog.** Node chỉ gửi lệnh khi nhận `/cmd_vel`, và driver ZLAC8015D giữ
  lệnh tốc độ cuối cùng. Nếu nguồn `cmd_vel` ngừng phát (hoặc tầng trên treo) mà node vẫn
  sống, xe cứ chạy tiếp đúng lệnh cuối cùng. Khi lên xe thật, bắt buộc bù bằng:
  (a) command-timeout/watchdog trên ZLAC8015D (tra datasheet), hoặc (b) watchdog ở phía
  node/tầng trên, và **nút E-stop cứng cắt nguồn động lực** độc lập với đường điều khiển.
  Đây là yêu cầu an toàn, không phải tùy chọn.

---

## 4. Tích hợp ROS 2 (`zlac_node.cpp`)

Ý tưởng cốt lõi: `ZlacDriver` và `ZlacEncoder` là lớp thuần C++ không phụ thuộc ROS,
nên node **bọc** chúng lại, không sửa chúng. `zlac_node.cpp` đóng vai cầu nối giữa
hệ ROS và phần cứng.

### 4.1. Kiến trúc: một tiến trình, hai node

`zlac_node` tạo hai node (class `rclcpp::Node` bình thường, **không** phải lifecycle
node) và đưa cả hai vào **một `SingleThreadedExecutor`**:

```
tiến trình zlac_node
 ├─ node /zlac_driver   sub /cmd_vel → ZlacDriver  (ghi tốc độ)  ─┐
 │                                                                ├─▶ /dev/ttyUSB_driver ─▶ ZLAC8015D
 └─ node /zlac_encoder  timer 50 Hz  → ZlacEncoder (đọc encoder) ─┘
                        → pub /encoder
```

**Vì sao gộp chung một tiến trình:** ZLAC8015D là một thiết bị trên một cổng, Modbus
RTU hỏi–đáp nên mỗi lúc trên dây chỉ có một lượt. Tách hai tiến trình không chạy song
song được gì, lại phải giữ khớp cấu hình (termios, dấu bánh) bằng tay ở hai nơi.
Executor đơn luồng chạy lần lượt callback `/cmd_vel` và timer encoder, nên hai node
không bao giờ hỏi–đáp chồng nhau.

Hai node vẫn riêng tên, logger, tham số, topic: `ros2 node list` thấy `/zlac_driver`
và `/zlac_encoder`. Nhưng không tắt riêng được bằng Ctrl+C — muốn ngừng encoder thì
dùng tham số `enabled` (mục 4.2).

**Thứ tự khởi động trong `main`:** dựng node driver trước (lỗi → log fatal, thoát cả
tiến trình). Sau đó mở cổng cho `ZlacEncoder` và dựng node encoder; nếu bước này lỗi
(không mở được cổng, không đọc được số vạch) thì chỉ log lỗi, **node driver vẫn chạy**
để điều khiển xe, chỉ là không có `/encoder`. Thứ tự này có ý nghĩa: `zlac_node` thứ
hai bị từ chối ngay ở bước dựng node driver (cổng đã bị khóa, mục 2) và thoát trước
khi kịp mở encoder.

`ZlacDriverNode`:

1. **Khởi tạo trong constructor.** `open()` + `set_velocity_mode()` + `enable()` được
   gọi ngay khi node dựng; thất bại thì `throw` (bắt ở `main`, log fatal rồi thoát).
2. **Sub `/cmd_vel`** (`geometry_msgs/msg/Twist`, queue 10). Mỗi message → `onCmdVel`:
   lấy `linear.x` (v) và `angular.z` (ω), gọi `send_velocity(v, ω, clamped)` ngay,
   cảnh báo nếu `clamped`. `cmd_vel` đã theo quy ước ROS (x tiến, z CCW) — khớp thẳng
   với thư viện, không cần đổi dấu ở node.
3. **Destructor:** `stop()` rồi `disable()` trước khi `ZlacDriver` tự `close()`.

Node driver không có watchdog: tần số gửi lệnh bằng tần số `/cmd_vel` đến.

`EncoderNode`: xem mục 4.2.

### 4.1.1. Lưu ý khi đọc/sửa code
- **Không dùng lifecycle node.** Không tách được "đã mở cổng" khỏi "đã cấp mô-men".
  Nếu cần bật/tắt động cơ mà không kill node (vd nút E-stop mềm qua service), đây là chỗ
  cần nâng cấp.
- **Gần như không dùng tham số ROS** — chỉ có `enabled` của node encoder. `device`,
  `wheel_base`, `wheel_radius`, `rpm_max`, `slave`... hard-code theo `ZlacConfig` mặc
  định; đổi phải sửa code + build lại.
- **Không có watchdog** (xem mục 3.6).
- **Khóa cổng chỉ chặn tiến trình khác.** `ZlacDriver::open()` khóa độc quyền cổng
  (mục 2), còn `ZlacEncoder` mở fd thứ hai trong cùng tiến trình và không khóa. Hai
  bên không hỏi–đáp chồng nhau chỉ vì executor đơn luồng cho chúng chạy lần lượt. Đổi
  sang `MultiThreadedExecutor` hay thêm luồng dùng cổng thì phải tự thêm khóa (vd
  `std::mutex`) quanh mỗi lượt hỏi–đáp.

### 4.2. Encoder → `/encoder` (node `zlac_encoder`)
Node chỉ **thu dữ liệu encoder và phát lên topic**; không tính odom, không phát TF
— việc đó thuộc khối khác.

**Đọc dữ liệu.** Mỗi chu kỳ gửi một frame FC 0x03 đọc 6 thanh ghi liền nhau từ
`0x20A7`, nên hai bánh được lấy mẫu cùng lúc:

    0x20A7, 0x20A8   vị trí bánh TRÁI (16 bit cao, 16 bit thấp) → int32, đơn vị counts
    0x20A9, 0x20AA   vị trí bánh PHẢI
    0x20AB, 0x20AC   tốc độ thực TRÁI, PHẢI (int16, đơn vị 0,1 r/min)

Lúc khởi động node đọc số vạch encoder ở `0x2030` (trái) và `0x2060` (phải);
`counts mỗi vòng = số vạch × counts_per_line` (mặc định 4, encoder vuông pha).
Trên xe đã đọc được 4096 vạch cả hai bánh → 16384 counts/vòng (hệ số ×4 là suy
ra, chưa đo).

**Message** `sensor_msgs/msg/JointState`, 50 Hz:

    header.stamp   thời điểm nhận trả lời từ driver
    name           ["left_wheel_joint", "right_wheel_joint"]
    position       góc mỗi bánh (rad), cộng dồn từ lúc node khởi động (bắt đầu từ 0)
    velocity       vận tốc góc mỗi bánh (rad/s), từ thanh ghi tốc độ thực
    effort         để trống

Cả `position` và `velocity` đã nhân `dir_left`/`dir_right`: **dương = xe tiến**, cho
cả hai bánh. Góc cộng dồn từ hiệu counts giữa hai lần đọc (trừ trên uint32 nên vẫn
đúng khi bộ đếm tràn):

    position += Δcounts × dir × 2π / (số_vạch × counts_per_line)
    velocity  = rpm_thực × dir × 2π / 60

Muốn ra quãng đường bánh (m) thì nhân `position` với `wheel_radius` (0,0825 m) —
phần này để khối nhận dữ liệu làm.

**Tham số `enabled`** (bool, mặc định `true`) — tắt/bật việc đọc encoder mà không
ảnh hưởng điều khiển bánh:
```bash
# lúc khởi động
ros2 run zlac_driver zlac_node --ros-args -p zlac_encoder:enabled:=false
# lúc đang chạy (xe không dừng)
ros2 param set /zlac_encoder enabled false
ros2 param set /zlac_encoder enabled true
```
- **Tắt:** dừng timer → không gửi FC 0x03, không phát `/encoder`; trên dây chỉ còn
  lệnh tốc độ. Node `/zlac_encoder` vẫn có trong `ros2 node list`.
- **Bật lại:** bộ đếm counts trong driver vẫn đếm lúc tắt, nên lần đọc đầu cộng luôn
  quãng bánh đã quay — `position` không về 0, không mất đoạn. Trong bag sẽ thấy
  `/encoder` có một khoảng trống rồi `position` nhảy một bước đúng bằng quãng đó.
- **Khởi động với `enabled:=false`:** node vẫn đọc số vạch một lần (để bật lại
  được); góc tính từ lúc bật lần đầu.
- Đặt sai kiểu (vd `enabled 5`) bị ROS từ chối, giá trị cũ giữ nguyên.

**Cần kiểm trên xe thật:**
- *Dấu:* code nhân `dir_left`/`dir_right` cho cả chiều đọc về, với giả định
  driver đếm counts tăng theo chiều +RPM. Cho xe tiến thì `position` của **cả
  hai** bánh phải tăng và `velocity` dương.
- *Hệ số ×4, counts ở velocity mode:* chạy thẳng 1 m, `position` mỗi bánh phải
  tăng ≈ 1 / 0,0825 ≈ 12,1 rad. Nếu `position` đứng yên thì driver không cập nhật
  counts ở velocity mode; nếu lệch theo tỉ lệ cố định thì sửa `counts_per_line`.

**Ghi rosbag:** xem quy trình đầy đủ ở mục 4.5.

### 4.3. Cấu trúc package
Xem cây thư mục ở mục 3.1. Core được build thành thư viện riêng `zlac_driver_lib` và
export qua `ament_export_targets`, nên package khác có thể `find_package(zlac_driver)`
và link target đó.

### 4.4. Test nhanh

Terminal 1: chạy node thật
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run zlac_driver zlac_node

Terminal 2: chạy teleop
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.3

Terminal 3 (tùy chọn): so vận tốc lệnh với vận tốc encoder đo được — **kê hai bánh
khỏi mặt đất**
```bash
source /opt/ros/jazzy/setup.bash
python3 ~/ros2_ws/src/zlac_driver/scripts/check_velocity.py
```
Script nghe `/cmd_vel` và `/encoder`, đổi cả hai về **vận tốc dài của từng bánh (m/s)**
và vẽ lại bảng 5 lần/giây (ví dụ dưới chạy với ZLAC giả lập, `speed:=0.05`, phím `i`):

                  ---------- LENH ----------   ---- DO (encoder) ----
                    rpm tinh -> gui       m/s    velocity   tu position   LECH (do - lenh)
      banh trai       +5.79 ->   +6   +0.0518     +0.0518       +0.0518   +0.0000 m/s  (+0.0%)
      banh phai       +5.79 ->   +6   +0.0518     +0.0518       +0.0518   +0.0000 m/s  (+0.0%)

- **LỆNH** tính giống driver: inverse kinematics → rpm → làm tròn số nguyên (cột
  `rpm tinh -> gui`) → m/s. Phải so với giá trị đã làm tròn: ở tốc độ thấp phần làm
  tròn đáng kể (0,05 m/s = 5,79 rpm nhưng gửi 6 rpm).
- **ĐO:** `velocity` × r (thanh ghi tốc độ của driver), và Δposition/Δt × r trong 1 s
  gần nhất (cột `tu position`).
- **LỆCH** = đo − lệnh (m/s); % chỉ hiện khi |rpm gửi| ≥ 3, tốc độ nhỏ hơn thì % không
  có nghĩa.
- Teleop chỉ phát khi bấm phím, driver giữ lệnh cuối; script cũng giữ lệnh cuối nên cột
  LỆNH luôn là tốc độ bánh đang nhận. Lệnh có |v| > `max_linear_speed` bị driver bỏ
  qua — script báo `!!` và vẫn hiện lệnh cũ. Bấm phím rồi đợi 2–3 s cho tốc độ ổn định
  mới đọc (node đặt thời gian tăng/giảm tốc 800 ở `0x2080–0x2083`).

| Hiện tượng | Ý nghĩa |
|---|---|
| Lệch vài %, dao động quanh 0 | Bình thường — vòng tốc độ của ZLAC |
| Một bánh **ngược dấu** | `dir` khi đọc về sai |
| Cột `velocity` lệch theo **tỉ lệ cố định** (×10, ÷10) | Đơn vị thanh ghi tốc độ không phải 0,1 r/min |
| Cột `tu position` lệch cột `velocity` theo tỉ lệ cố định (×4, ÷4) | `counts_per_line` sai |
| Bánh quay mà `velocity` = 0 | Đọc sai thanh ghi |

Test này không kiểm `wheel_radius`/`wheel_base` (bánh kê, không lăn trên sàn) — xem mục
4.5. Thông số xe của script (`wheel_base`, `wheel_radius`, `rpm_max`, `max_linear_speed`)
là tham số ROS, mặc định **chép** từ `ZlacConfig`: sửa `zlac_driver.hpp` thì truyền theo,
vd `--ros-args -p wheel_radius:=0.085`.

### 4.5. Thu dữ liệu `/encoder` trên xe thật (rosbag)

Máy nhúng trên xe: `vistar@192.168.0.101`, workspace `~/ros2_ws`. Bước 0 và bước
cuối chạy trên laptop; các bước còn lại mỗi bước một cửa sổ ssh riêng tới máy nhúng.

**Bước 0 — chép code sang máy nhúng (trên laptop):**
```bash
rsync -avz --progress /home/billngo/vkist/amr/driver_banhxe/zlac_driver vistar@192.168.0.101:~/ros2_ws/src/
```
Nếu trên máy nhúng còn package encoder cũ (`zlac_encoder` hoặc `zlac_encorder`)
trong `~/ros2_ws/src`, xóa nó cùng thư mục tương ứng trong `build/` và `install/` —
encoder giờ nằm trong `zlac_node`, chạy thêm bản cũ thì `/encoder` có hai nguồn phát.

**Bước 1 — build và chạy node:**
```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select zlac_driver
source install/setup.bash
ros2 run zlac_driver zlac_node
```
Node chạy đúng sẽ in hai dòng `zlac_driver san sang, dang nghe /cmd_vel` và
`zlac_encoder san sang, phat /encoder (encoder 4096/4096 vach)`. Nếu chỉ thấy dòng
thứ nhất kèm lỗi `[zlac_encoder]` thì xe vẫn điều khiển được nhưng không có `/encoder`.

**Nếu cổng đang bị người khác giữ**, node thoát ngay với lỗi:

    [zlac] /dev/ttyUSB_driver dang bi chuong trinh khac giu (zlac_node khac?) - kiem: pgrep -af zlac_node

nghĩa là đã có `zlac_node` khác đang chạy (vd người khác ssh vào chạy trước). Bản của
bạn chưa gửi frame nào nên không ảnh hưởng bản kia. Muốn chạy bản của mình:
1. Tìm bản đang chạy: `pgrep -af zlac_node` (xem ai đang đăng nhập: `who`). Nếu người
   khác đang dùng xe thì báo họ trước.
2. Tắt bản đó: Ctrl+C trong cửa sổ đang chạy nó, hoặc từ ssh của mình
   `pkill -INT -x zlac_node` (giống Ctrl+C: node gửi stop + disable rồi mới thoát).
   **Không** dùng `kill -9` — node chết mà không gửi stop, ZLAC giữ tốc độ cuối và xe
   chạy tiếp.
3. Chạy lại `ros2 run zlac_driver zlac_node`. Khóa được nhả ngay khi bản kia thoát.

Nếu node chạy nhưng log liên tục báo `khong co ACK` / `frame tra loi hong`, ngoài lỗi
dây/nguồn thì kiểm xem có chương trình không dùng khóa (minicom, script thử...) đang
mở cổng không: `fuser -v /dev/ttyUSB_driver`.

**Bước 2 — kiểm `/encoder` trước khi ghi:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /encoder                    # phải ra khoảng 50 Hz
ros2 topic info /encoder                  # Publisher count: 1 (2 = còn bản cũ đang chạy)
ros2 topic echo /encoder --field position # theo dõi góc 2 bánh
```

**Bước 3 — ghi bag** (bắt đầu ghi trước khi lái, ghi xong bấm Ctrl+C):
```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/bags && cd ~/bags
ros2 bag record -o encoder_$(date +%Y%m%d_%H%M%S) /encoder
```

**Bước 4 — lái xe:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.3
```
Node không có timeout cho `/cmd_vel` (mục 3.6): bấm `i` một lần là xe chạy mãi
cho tới khi bấm `k`. Nếu ssh rớt giữa chừng xe vẫn chạy tiếp, nên giữ nút dừng
khẩn cấp trong tầm tay.

**Bước 5 — kiểm bag (trên máy nhúng) rồi chép về (trên laptop):**
```bash
ros2 bag info ~/bags/encoder_<tên vừa ghi>
```
Phải thấy `/encoder` với số message xấp xỉ 50 × số giây ghi.
```bash
rsync -avz --progress vistar@192.168.0.101:~/bags/ ~/vkist/amr/bags/
```
Gửi cho nhóm SLAM nguyên thư mục bag, gồm `metadata.yaml` và file `.mcap`.

**Thứ tự nên làm lần đầu** (code đọc encoder chưa được kiểm trên xe thật):
1. Kê hai bánh khỏi mặt đất, bấm `i`: `position` cả hai bánh phải tăng. Bấm `j`
   (xoay trái tại chỗ): bánh trái giảm, bánh phải tăng. Nếu ngược thì dấu đọc về
   sai (mục 4.2).
2. Hạ xe, chạy thẳng 1 m theo thước: `position` mỗi bánh phải tăng khoảng 12,1 rad.
3. Hai bước trên đúng thì mới ghi bag chính thức để gửi.

---

## 5. Đề xuất
1. **(xong)** Bring-up phần cứng, chốt quy ước dấu + mapping word→bánh.
2. **(xong)** Đóng gói core `ZlacDriver`, lớp giao tiếp mức AMR.
3. **(xong)** Bọc thành `zlac_node` ROS 2 (sub `cmd_vel` → `send_velocity`).
4. **(xong)** Cấu trúc package ament_cmake (`package.xml` + `CMakeLists.txt`), build được bằng `colcon`.
5. Thêm `declare_parameter` vào `zlac_node.cpp` thay cho `ZlacConfig` mặc định, để đổi
   `device`/`wheel_base`/... qua YAML mà không cần build lại (hiện mới có `enabled`
   của node encoder).
6. Thêm watchdog: node tự stop khi quá N giây không có `/cmd_vel`, và/hoặc bật
   command-timeout **trên driver** (tra datasheet).
7. **(xong)** Đọc encoder (FC 0x03), phát `/encoder` (`JointState`); tính odom do
   khối khác làm (mục 4.2).
8. **(xong)** Gộp điều khiển bánh và encoder vào một tiến trình `zlac_node` (hai node,
   một executor), tham số `enabled` để tắt/bật encoder (mục 4.1, 4.2).
9. Tách lớp cổng Modbus chung (mở cổng + termios, CRC, `read_exact`) để `ZlacDriver`
   và `ZlacEncoder` dùng chung một fd — hiện phần này còn chép hai bản trong
   `zlac_driver.cpp` và `zlac_encoder.cpp`.
10. **(xong)** Khóa độc quyền lúc mở cổng để chặn chạy 2 bản `zlac_node` cùng lúc (vd
    hai người ssh vào cùng chạy): `flock(LOCK_EX | LOCK_NB)` một lần trong
    `ZlacDriver::open()`, giữ đến khi đóng (mục 2, cách xử lý ở mục 4.5). Khi làm lớp
    cổng chung (mục 9) thì chuyển khóa sang lớp đó.
