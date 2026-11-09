# ZLAC Motor Web — ứng dụng độc lập

Giao diện điều khiển và kiểm tra động cơ ZLAC8015D bằng ROS 2.
**Không thay đổi bất kỳ file nào của `../zlac_driver_encoder`.** Web, cầu nối,
watchdog và sản phẩm build nằm trong thư mục này. Thuật toán joypad được tách
thành module độc lập tại [`../joypad_driver`](../joypad_driver/README.md).

## Cấu trúc và luồng dữ liệu

```text
HMI_demo/
├── zlac_driver_encoder/         # package gốc, giữ nguyên
├── joypad_driver/               # module điều khiển đầu vào độc lập
│   ├── src/joypad_driver.js     # chuột/cảm ứng/WASD → (v,ω), vùng chết, dừng
│   ├── tests/                  # kiểm thử thuật toán và sự kiện riêng
│   └── docs/THUYET_MINH.md      # tài liệu dùng làm cơ sở báo cáo công việc 3.6
└── zlac_motor_web/
    ├── app.py                  # HTTP server + publisher /cmd_vel + subscriber /encoder
    ├── guard.py                # node ROS độc lập: watchdog, validate, remap, giám sát process
    ├── web_core.py             # động học, giới hạn, lease và độ mới của lệnh
    ├── static/                 # HTML, CSS, adapter hiển thị/HTTP; không npm/CDN
    ├── tools/build_driver.py   # build package gốc vào .runtime, đọc cấu hình thực
    ├── tools/driver_child.py   # Linux parent-death signal cho driver con
    ├── docs/ANALYSIS.md         # phân tích code gốc, đơn vị/dấu/rủi ro
    ├── tests/                  # unit + ROS/Modbus PTY integration
    └── .runtime/               # build/install/log/manifest; được tạo khi build
```

```text
Chuột / cảm ứng / WASD → joypad_driver → static/app.js → HTTP cùng origin → app.py
    → /cmd_vel (geometry_msgs/Twist)
    → guard.py (watchdog ROS)
    → /zlac_guard_<id>/cmd_vel (topic riêng mỗi lần chạy)
    → zlac_node nguyên bản (remap /cmd_vel sang topic riêng)
    → RS485 → động cơ

zlac_node nguyên bản → /encoder (sensor_msgs/JointState)
    → app.py → trình duyệt

guard.py → /zlac/status (std_msgs/String JSON)
    → app.py: cấu hình thực và trạng thái chuyển tiếp, không phải ACK phần cứng
```

Web và guard **không mở cổng serial**. Guard tự khởi động đúng binary driver đã
build từ nguồn gốc. Không chạy thêm `ros2 run zlac_driver zlac_node` hoặc teleop
song song. Driver nguyên bản chỉ nhận topic riêng, không trực tiếp nhận `/cmd_vel`.

Server phục vụ trực tiếp `../joypad_driver/src/joypad_driver.js` tại URL
`/joypad_driver/joypad_driver.js`; không sao chép mã vào `static`. Giữ hai thư
mục ngang cấp khi di chuyển/cài đặt. Module joypad chạy trong trình duyệt,
không cần terminal, npm hay bước build riêng.

**Cập nhật từ bản web cũ:** khi xe đã dừng, Ctrl+C terminal `app.py`, chạy lại
lệnh web như dưới đây rồi Ctrl+F5 trình duyệt. Server cũ chưa có route của
module mới; chỉ refresh sẽ không đủ. Không cần build lại driver/encoder cho
việc tách joypad. Nếu thiếu source module, server mới báo lỗi ngay khi khởi động.

## Chạy demo — không cần ROS, không cần động cơ

Cần Python 3.10+; giao diện dùng trình duyệt hiện đại hỗ trợ Pointer Events,
Fetch và AbortSignal.timeout. Không cần cài thư viện Python bên ngoài cho demo.

```bash
cd /home/vistar/projects/HMI_demo/zlac_motor_web
python3 app.py --demo
```

Mở **http://127.0.0.1:8080**. Nhãn **DEMO — DỮ LIỆU GIẢ LẬP** luôn hiện.
Demo không import ROS, không phát topic và không truy cập thiết bị. Phản hồi có
độ trễ mô phỏng; không dùng để đánh giá khả năng thực của động cơ. Nút “Thử mất
encoder” bật/tắt luồng giả lập để kiểm tra báo quá hạn và khóa điều khiển.

## Cài đặt và chạy với ROS 2

Đã kiểm thử build trên Linux, ROS 2 Jazzy, Python 3.12. Cần `rclpy`, `rclcpp`,
`geometry_msgs`, `sensor_msgs`, `std_msgs`, `colcon`, `ament_cmake`, C++ compiler.
Nếu máy chưa có đủ dependency, cài qua ROS/Ubuntu trước:

```bash
sudo apt install python3-colcon-common-extensions build-essential \
  ros-jazzy-ament-cmake ros-jazzy-rclcpp ros-jazzy-rclpy \
  ros-jazzy-geometry-msgs ros-jazzy-sensor-msgs ros-jazzy-std-msgs
```

**Kê hai bánh khỏi mặt đất, chuẩn bị E-stop trước khi chạy guard.** Driver gốc
sẽ enable động cơ trong constructor. Kiểm tra không còn tiến trình khác giữ
cổng `/dev/ttyUSB_driver` và tài khoản có quyền truy cập cổng như README gốc.

Terminal 1, build và chạy node giám sát (node này tự chạy driver + encoder):

```bash
source /opt/ros/jazzy/setup.bash
cd /home/vistar/projects/HMI_demo/zlac_motor_web
python3 tools/build_driver.py
python3 guard.py --command-timeout 0.5 --encoder-timeout 0.5
```

Terminal 2, chạy web thật:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/vistar/projects/HMI_demo/zlac_motor_web
python3 app.py --web-timeout 0.25 --encoder-timeout 0.5
```

Mở http://127.0.0.1:8080. Chỉ cho điều khiển khi có cấu hình guard hợp lệ,
heartbeat mới, encoder mới và chỉ một nguồn phát mỗi topic liên quan.
Đóng web bằng Ctrl+C, rồi đóng guard bằng Ctrl+C. Guard yêu cầu zero và SIGINT
để destructor driver nguyên bản gửi stop + disable.

Với package gốc ở đường dẫn khác:

```bash
python3 tools/build_driver.py --source /duong/dan/toi/zlac_driver_encoder
```

Helper chỉ **đọc** source rồi build vào `.runtime` của web. Manifest lưu hash
nguồn và binary; guard kiểm tra lại trước khi chạy. Nếu source/binary thay đổi,
phải build lại. Cấu hình lấy từ `ZlacConfig` trong header của đúng source đó,
không cho nhập hình học tùy ý trên trình duyệt. Không dùng binary ROS cũ có
cấu hình khác với giao diện. `io_timeout_ms=1000` hiển thị trong API là timeout
đọc **gốc**, không phải tham số có thể thay đổi của guard.

Để mở từ laptop khác, giữ server ở loopback và dùng SSH tunnel:

```bash
ssh -N -L 8080:127.0.0.1:8080 robot-user@robot-host
```

Nếu cần truy cập cảm ứng qua LAN tin cậy, có thể chạy:

```bash
python3 app.py --host 0.0.0.0 --allowed-host 192.168.0.101:8080
```

Thay IP bằng địa chỉ thực của máy ROS, rồi mở `http://IP:8080`. Đây là giao diện
điều khiển **không có đăng nhập**; chỉ dùng mạng thử nghiệm được kiểm soát, không
đưa ra Internet. Mặc định chỉ bind loopback; kiểm Origin/Host và token theo tab
chặn yêu cầu từ website khác, không thay thế xác thực người vận hành.

## Điều khiển và ý nghĩa số đo

Nhập **độ lớn không âm** cho tiến/lùi (m/s), trái/phải (rad/s). Mặc định 0,10 m/s
và 0,30 rad/s. Joystick chạy khi giữ chuột hoặc chạm, dọc tạo `linear.x`, ngang
tạo `angular.z`; có thể kết hợp. Đẩy lên = tiến, kéo xuống = lùi, kéo trái =
`angular.z > 0`. Độ lệch tỷ lệ với tốc độ; hướng chéo được chuẩn hóa trong hình
tròn. Vùng chết tâm 5%.

**Bàn phím WASD:** nhấp vào vùng trống của trang (ra khỏi ô nhập), rồi giữ
`W` để tiến, `S` để lùi, `A` để quay trái, `D` để quay phải. Có thể giữ `W+A`,
`W+D`, `S+A`, `S+D` để vừa đi vừa quay. Tốc độ dùng bốn ô nhập tương ứng;
hai trục chéo được chuẩn hóa giống joystick. Các phím đối nhau triệt tiêu nhau.
Thả hết WASD, nhấn `Esc` hoặc nút Dừng để yêu cầu zero. Sau Dừng/mất kết nối,
phải thả và nhấn lại; auto-repeat của phím đang giữ không tự khởi động lại.
Không điều khiển khi đang gõ vào ô nhập hoặc dùng tổ hợp Ctrl/Alt/Meta.
Mất focus, ẩn tab, thay đổi giá trị hoặc chuyển sang nhập liệu đều yêu cầu dừng.

Cách cũ vẫn dùng được: focus joystick, giữ Space và các phím mũi tên; thả
Space để dừng. Không trộn điều khiển chuột/cảm ứng, WASD và Space + mũi tên
trong cùng một lần giữ.

Cấu hình gốc: `L=0,40 m`, `r=0,0825 m`, `rpm_max=300`, `max_linear_speed=3 m/s`.
Guard giới hạn `|ω| ≤ 2 × rpm_max × 2π/60 × r/L ≈ 12,959 rad/s`.
Đây là giới hạn toán học, không phải khuyến nghị tốc độ thử phần cứng.

Với mỗi bánh:

```text
v_trái  = v − ωL/2
v_phải  = v + ωL/2
rpm_tính = v_bánh / r × 60/(2π)
rpm_gửi  = lround(clamp(rpm_tính, −rpm_max, +rpm_max))
v_dự_kiến = rpm_gửi × 2π/60 × r
v_đo = JointState.velocity[joint] × r
sai_lệch = v_đo − v_dự_kiến
```

`lround` làm tròn nửa ra xa 0, giống C++ và `check_velocity.py`.
Ví dụ 0,05 m/s → 6 RPM → 0,051836 m/s. Hai joint được tìm bằng **tên**,
không dựa vào thứ tự array. Encoder đã bù dấu; không nhân `dir_right` lần nữa.
Cắt RPM từng bánh có thể làm sai độ cong yêu cầu; giao diện báo khi bị cắt.

- “Yêu cầu từ joystick”: ý định hiện tại của người dùng.
- “Lệnh đã phát”: cầu nối đã phát Twist; không chứng minh guard/driver thực hiện.
- “Giám sát”: guard đã chuyển tiếp hoặc đã yêu cầu zero. Driver gốc không phát
  ACK lên ROS; không có bằng chứng ACK Modbus trên web.
- “Đo được”: encoder mới, quy đổi m/s. Sai lệch so với mục tiêu sau làm tròn,
  có thể khác khi tăng/giảm tốc. Encoder không đo trượt bánh hoặc tốc độ thân xe.
- Mẫu thiếu, malformed, NaN, thiếu/trùng joint, stamp cũ/lặp hoặc quá hạn đều
  không được hiển thị là số đo mới. Có giờ nhận, stamp ROS và tuổi dữ liệu.
  Trình duyệt mất cầu nối sẽ xóa số đo hiện tại, không giả vờ bánh đứng yên.

## Các lớp dừng và giới hạn còn lại

1. Frontend phát zero khi thả/cancel joystick, mất focus, ẩn tab, rời trang,
   mất mạng hoặc nhấn Dừng. Dùng beacon khi trang sắp đóng; beacon có thể thất bại.
2. Cầu nối kiểm heartbeat (mặc định 250 ms), thu hồi lease, phát zero. Chỉ một
   tab được điều khiển. Nút Dừng từ tab hợp lệ có thể thu hồi tab khác. Sequence
   và ticket server ngắn hạn ngăn lệnh cũ/quá hạn khởi động lại; phải nhấn lại.
3. **Node ROS guard độc lập**: mặc định 500 ms không nhận `/cmd_vel` mới → yêu
   cầu zero trên topic riêng. Đồng hồ monotonic, kiểm mỗi 20 ms. Lệnh không hợp
   lệ hoặc encoder hết hạn cũng yêu cầu zero; tiếp tục phát zero khi không active.
   Không tự phát lại một lệnh chuyển động cũ để “giữ sống” driver.
4. Khi cầu nối bị SIGKILL, guard vẫn chạy. Khi guard chết, Linux
   `PR_SET_PDEATHSIG=SIGINT` yêu cầu tiến trình driver con thoát mềm. Có kiểm tra
   race khi khởi động để không chạy driver nếu guard cha đã chết.

**Giữ nguyên code gốc đồng nghĩa không thể bảo đảm dừng trong 500 ms.** Driver
và encoder cùng executor; đọc serial bản gốc có thể chặn 1 s mỗi lượt, frame
về từng phần còn kéo dài hơn. Stop phải chờ callback và driver giảm tốc. Nếu
OS/driver process chết hoặc RS485 mất, zero và SIGINT có thể không tác dụng.
Lệnh enable ở startup cũng giữ đúng hành vi gốc. Không coi nút Dừng web là
E-stop, không coi yêu cầu zero là bằng chứng bánh đã dừng.

Guard không sửa firmware, không cấu hình watchdog phần cứng, không xác nhận
an toàn chức năng. Cần E-stop phần cứng độc lập. Các kiểm thử phần mềm dưới đây
không thay cho việc đo thời gian dừng và kiểm tra chiều quay trên xe.

## Quy trình thử trên bánh đã kê khỏi mặt đất

1. Kiểm tra giá kê chắc chắn, vùng quay trống, nút E-stop hoạt động. Bắt đầu
   với tiến/lùi 0,05 m/s, trái/phải 0,20 rad/s. Không thử giới hạn 300 RPM lần đầu.
2. Chạy guard rồi web. Kiểm tra đúng nhãn phần cứng, cấu hình hình học, encoder
   mới và số đo gần zero. Nếu không có encoder thì không được arming joystick.
3. Giữ tiến nhẹ: cả hai số đo phải dương. Lùi: cả hai âm. Quay trái tại chỗ:
   trái âm/phải dương. Quay phải: trái dương/phải âm. Nếu sai dấu, Dừng và E-stop;
   không bù dấu tùy tiện trên web để che lỗi cấu hình/phần cứng.
4. Kiểm tra vừa đi vừa quay ở tốc độ thấp; đối chiếu RPM nguyên và m/s dự kiến.
   Chờ ổn định để đọc sai lệch, không dùng sai lệch tức thời khi tăng tốc làm kết luận.
5. Lần lượt thử thả chuột/ngón tay, kéo ra ngoài vùng, nút Dừng, chuyển cửa sổ,
   ẩn tab và đóng trang. Xác nhận encoder giảm về gần zero và quan sát bánh thật.
6. Thử ngắt kết nối web/SSH khi tốc độ rất thấp; guard vẫn phải yêu cầu dừng.
   Quan sát topic riêng ghi trong `/zlac/status`. Kiểm tra ngừng cầu nối đột ngột
   khi sẵn sàng dùng E-stop. Đo thời gian thực, không suy từ timeout cấu hình.
7. Tắt encoder bằng `ros2 param set /zlac_encoder enabled false`: web/guard phải
   báo quá hạn và yêu cầu zero. Bật lại bằng `true`; không tự khởi động lại.
8. Xác minh mất dây RS485/E-stop theo quy trình an toàn phần cứng của đội vận hành.
   Không giả định software watchdog xử lý được đứt bus hoặc driver treo.
9. Sau mọi bước, nếu chưa thấy encoder/quan sát xác nhận dừng thì dùng E-stop.
   Hoàn tất bằng Ctrl+C web, Ctrl+C guard, xác minh bánh dừng và driver disable.

## Kiểm thử tự động

```bash
cd /home/vistar/projects/HMI_demo/zlac_motor_web
python3 -m unittest discover -s tests -p test_web_core.py -v
python3 -m unittest discover -s tests -p test_http.py -v
source /opt/ros/jazzy/setup.bash
python3 tools/build_driver.py
python3 -m unittest discover -s tests -p test_ros_integration.py -v
```

Kiểm thử sự kiện bàn phím bằng JavaScript (GJS chỉ cần khi chạy test, không cần
để chạy web; máy hiện tại đã có):

```bash
gjs ../joypad_driver/tests/test_joypad_driver.js
gjs tests/test_keyboard.js
```

12 ca độc lập kiểm tra thuật toán và vòng đời của module joypad. 16 trường hợp
kiểm thử frontend nạp `joypad_driver` rồi `static/app.js` với DOM và HTTP giả:
W/S/A/D, kết hợp/phím đối nhau, thả từng phím, ô nhập/IME/phím tắt, tốc độ sai,
Dừng/Escape và auto-repeat, thả phím khi đang chờ arming, mất focus/ẩn tab,
chuyển focus vào ô nhập, không trộn chuột với WASD, giữ tương thích Space + mũi
tên, mất kết nối, pointer cancel/mất capture, đổi tốc độ khi chạy và timeout
phản hồi. Không phát ROS hoặc gửi lệnh đến phần cứng.


Integration dùng ROS domain **176**, discovery localhost, HTTP port 8081 và
serial PTY. Một thư viện LD_PRELOAD **chỉ trong test** chuyển đường dẫn cổng
nguyên bản sang `/dev/pts/...`; không sửa source và không truy cập RS485 thật.
Chạy test khi domain 176/port 8081 rảnh. Không chạy thử này trong ROS domain của xe.

Đã chạy trong môi trường hiện tại:

- Build package nguyên bản bằng ROS 2 Jazzy thành công.
- 10 unit tests qua: làm tròn/động học khớp script gốc, dấu và tên joint, giới hạn,
  dữ liệu lỗi, timeout, lệnh đến trễ, sequence fence, một chủ điều khiển.
- Integration qua với **binary driver nguyên bản + guard + bridge**: vận tốc đúng
  dấu/quantization, watchdog ROS, lệnh NaN/Infinity/vượt giới hạn, explicit stop,
  mất HTTP heartbeat, mất encoder, SIGKILL cầu nối, SIGKILL guard/parent-death stop.
- 4 HTTP tests qua: bảo vệ Origin/Host, arm/command/stop, cảnh báo giới hạn,
  mô phỏng encoder mất/khôi phục, tải source joypad đúng thứ tự và báo thiếu
  module. Tổng cộng **15 Python tests qua** (10 core + 4 HTTP + 1 ROS/PTY).
- Sau khi tách module joypad: **12 test module + 16 test frontend qua** bằng
  GJS, cùng 15 Python tests trên; tổng cộng **43 ca kiểm thử qua**.
- Python syntax compile qua. Đã kiểm tra hiển thị và ô nhập trong trình duyệt demo;
  thao tác click/drag qua công cụ trình duyệt nhúng chưa xác minh được. Lần kiểm
  tra cuối công cụ trình duyệt từ chối truy cập trang demo theo chính sách công
  cụ. Kiểm thử API không thay thế kiểm thử chuột/cảm ứng thực tế.
- `git diff -- zlac_driver_encoder` trống; checksum các file nguồn driver/encoder
  không đổi sau khi tách joypad.

**Chưa xác minh trên phần cứng:** cổng thật, chiều lắp/dấu encoder, độ chính xác
bán kính/khoảng cách bánh, độ trễ dừng, hiệu quả giảm tốc, lỗi điện/RS485 thực,
watchdog firmware và E-stop. Cảm ứng dùng Pointer Events nhưng cần kiểm tra thêm
trên thiết bị cảm ứng thực tế. Phân tích chi tiết xem [docs/ANALYSIS.md](docs/ANALYSIS.md).
