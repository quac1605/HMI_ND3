# Joypad Driver — bộ xử lý đầu vào điều khiển robot

Mã nguồn tách riêng cho đầu mục **“Công việc 3.6: Viết chương trình driver giữa
joypad và robot”**. Module nhận thao tác joystick trên web hoặc bàn phím, xử lý
hướng/độ lệch, rồi tạo lệnh vận tốc thân xe `(v, ω)` cho cầu nối ROS hiện có.

Phiên bản này hỗ trợ **joypad ảo bằng chuột/cảm ứng, WASD và Space + mũi tên**.
Đây là lớp xử lý đầu vào ở mức ứng dụng; chưa có đọc tay cầm USB/Bluetooth,
`sensor_msgs/Joy` hay driver HID. Không ghi các chức năng đó là đã hoàn thành
trong báo cáo. Module không tự phát ROS và không mở cổng RS485.

## Mã nguồn và tài liệu

```text
HMI_demo/
├── joypad_driver/                 # thư mục này, không cần npm hoặc build
│   ├── src/joypad_driver.js        # thuật toán, kiểm tra tốc độ, xử lý sự kiện
│   ├── tests/test_joypad_driver.js # kiểm thử độc lập, không cần web/ROS
│   ├── docs/THUYET_MINH.md         # kiến trúc, thuật toán, trạng thái, phạm vi
│   └── README.md
├── zlac_motor_web/                # UI, HTTP–ROS, watchdog, hiển thị encoder
└── zlac_driver_encoder/           # package nguyên bản, không sửa
```

Web nạp trực tiếp `src/joypad_driver.js` bằng URL cùng origin
`/joypad_driver/joypad_driver.js`, trước khi nạp `static/app.js`. Không có bản
sao thuật toán trong web; sửa source tại đây là thay đổi bản web sẽ dùng.
Khi sao chép dự án, giữ `joypad_driver` và `zlac_motor_web` **ngang cấp**.

## Chạy với web hiện tại

Không cần tiến trình hay terminal riêng cho joypad: module chạy trong trình
duyệt. Lệnh chạy web và ROS giữ nguyên, xem
[hướng dẫn của web](../zlac_motor_web/README.md).

**Lần đầu cập nhật sang cấu trúc này:** khi xe đã dừng, Ctrl+C terminal chạy
`app.py`, chạy lại lệnh dưới đây rồi Ctrl+F5 trang web. Server cũ chưa có route
của module mới nên chỉ refresh trình duyệt là chưa đủ. Không chạy hai `app.py`
trên cùng cổng. Không cần build lại hoặc sửa package driver/encoder.

```bash
source /opt/ros/jazzy/setup.bash
cd /home/vistar/projects/HMI_demo/zlac_motor_web
python3 app.py --web-timeout 0.25 --encoder-timeout 0.5
```

Demo độc lập, không cần ROS hay phần cứng:

```bash
cd /home/vistar/projects/HMI_demo/zlac_motor_web
python3 app.py --demo --port 8082
```

Mở [demo trên cổng 8082](http://127.0.0.1:8082). Phải thấy nhãn **DEMO — DỮ LIỆU
GIẢ LẬP**. Cổng 8082 cho phép thử demo mà không đụng server thật ở 8080.

## Giao tiếp của module

Source đăng ký `globalThis.JoypadDriver`; không cần DOM khi chỉ dùng các hàm
tính toán. Các giá trị tốc độ phải là số JavaScript, không phải chuỗi nhập liệu.

| Hàm | Kết quả / trách nhiệm |
| --- | --- |
| `normalizeAxes(x, y, deadzone=0.05)` | Trục trong đĩa đơn vị; vùng chết quanh tâm. Trục không hữu hạn bị từ chối. |
| `validateSpeeds(speeds, limits)` | `{valid, fields}`; kiểm tra riêng `forward`, `backward`, `left`, `right`. |
| `toCommand(axes, speeds, limits)` | `{v, w}` tương ứng `Twist.linear.x` m/s, `Twist.angular.z` rad/s. Tốc độ sai ném `RangeError`. |
| `create(options)` | Gắn sự kiện chuột/cảm ứng/bàn phím và sự kiện dừng vào các đối tượng được truyền vào. |

Ví dụ thuật toán thuần sau khi nạp source:

```javascript
const speeds = {forward: 0.1, backward: 0.1, left: 0.3, right: 0.3};
// Trong ứng dụng, lấy limits từ cấu hình thực do cầu nối cung cấp.
const limits = {max_linear_speed: 3, max_angular_speed: 12.9};
const axes = JoypadDriver.normalizeAxes(-1, 1); // W + A: đi tiến và quay trái
const command = JoypadDriver.toCommand(axes, speeds, limits);
// command.v ≈ 0.070711 m/s; command.w ≈ 0.212132 rad/s.
// Ví dụ chỉ tính số; không gửi lệnh cho robot.
```

`create(options)` nhận các adapter sau; xem cách dùng thực trong
[`static/app.js`](../zlac_motor_web/static/app.js):

| Option | Hợp đồng |
| --- | --- |
| `surface` | Phần tử joystick có focus, Pointer Events và pointer capture. |
| `eventWindow`, `eventDocument` | Nguồn sự kiện bàn phím, focus, visibility, offline. |
| `canStart()` | Kiểm tra phiên, cấu hình, tốc độ và trạng thái sẵn sàng của robot. Trả boolean. |
| `onStart()` | Bắt đầu yêu cầu cấp quyền qua cầu nối; chưa được gửi vận tốc cho đến khi có lease hợp lệ. |
| `onMove({x,y})` | Cập nhật hiển thị ý định. Không phải xác nhận thực hiện lệnh. |
| `onStop(reason, beacon)` | Sau khi module đã về zero, adapter gửi yêu cầu dừng. `beacon=true` cho rời trang/mất focus/mất mạng. |
| `radius=71`, `deadzone=0.05` | Bán kính kéo theo CSS pixel và bán kính vùng chết chuẩn hóa. |

Đối tượng trả về có `holding`, `mode` (`pointer`, `wasd`, `arrows` hoặc `null`),
`axes`, `command(speeds, limits)`, `stop(reason, beacon)` và `dispose()`.
`command` trả zero khi không giữ. `dispose` tháo listener và yêu cầu dừng.

Adapter web giữ sequence, ticket, lease và bộ đếm `stopGeneration`: phản hồi
arm về sau khi đã dừng không được làm robot chạy lại. Module chỉ quản lý trạng
thái thao tác cục bộ; `holding=true` không có nghĩa robot đã cấp quyền hoặc chạy.
Adapter phải gọi `stop()` khi mất dữ liệu/lease, đổi tốc độ hoặc timeout mạng.
Watchdog HTTP/ROS phải tiếp tục chạy độc lập nếu trình duyệt bị treo.

## Kiểm thử

GJS chỉ dùng để chạy test, không phải dependency khi vận hành web:

```bash
cd /home/vistar/projects/HMI_demo
gjs joypad_driver/tests/test_joypad_driver.js
cd zlac_motor_web
gjs tests/test_keyboard.js
python3 -m unittest discover -s tests -p test_web_core.py -v
python3 -m unittest discover -s tests -p test_http.py -v
```

Các bài test JavaScript dùng sự kiện/DOM giả; HTTP dùng demo trên cổng loopback
tạm thời. Chúng không điều khiển phần cứng. Kiểm thử ROS/PTY và quy trình kê bánh
nằm trong [README của web](../zlac_motor_web/README.md#kiểm-thử-tự-động).

Thuyết minh thuật toán và danh mục kiểm chứng:
[`docs/THUYET_MINH.md`](docs/THUYET_MINH.md).
