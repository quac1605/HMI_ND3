// zlac_driver.cpp — implementation cua ZlacDriver.

#include "zlac_driver/zlac_driver.hpp"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/file.h>
#include <errno.h>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <algorithm>

namespace {
constexpr uint16_t REG_MODE     = 0x200D;
constexpr uint16_t VAL_VEL_MODE = 0x0003;
constexpr uint16_t REG_ENABLE   = 0x200E;
constexpr uint16_t VAL_ENABLE   = 0x0008;
constexpr uint16_t VAL_DISABLE  = 0x0007;
constexpr uint16_t REG_VEL_ADDR = 0x2088;   // word1=TRAI(0x2088), word2=PHAI(0x2089)


uint16_t modbus_crc(const uint8_t* d, int len) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

int read_exact(int fd, uint8_t* buf, int want) {
    int got = 0;
    while (got < want) {
        int n = ::read(fd, buf + got, want - got);
        if (n <= 0) break;   // 0 = timeout (het VTIME), <0 = loi
        got += n;
    }
    return got;
}
} // namespace

ZlacDriver::ZlacDriver(const ZlacConfig& cfg) : cfg_(cfg) {}

ZlacDriver::~ZlacDriver() { close(); }

bool ZlacDriver::open() {
    fd_ = ::open(cfg_.device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        fprintf(stderr, "[zlac] khong mo duoc %s: %s\n", cfg_.device, strerror(errno));
        return false;
    }
    // Khoa doc quyen den khi close(): chuong trinh thu 2 (vd zlac_node khac do nguoi khac
    // ssh vao chay) bi tu choi ngay day, truoc khi gui frame nao. Kernel tu nha khoa khi
    // dong fd / tien trinh thoat, ke ca crash hay kill -9.
    // ZlacEncoder mo fd rieng trong cung tien trinh va KHONG khoa (khoa se bi chinh khoa nay chan).
    if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
        const int err = errno;
        if (err == EWOULDBLOCK)
            fprintf(stderr, "[zlac] %s dang bi chuong trinh khac giu (zlac_node khac?)"
                            " - kiem: pgrep -af zlac_node\n", cfg_.device);
        else
            fprintf(stderr, "[zlac] khong khoa duoc %s: %s\n", cfg_.device, strerror(err));
        close();
        return false;
    }
    struct termios tio;
    if (tcgetattr(fd_, &tio) != 0) { close(); return false; }
    cfsetispeed(&tio, B115200); cfsetospeed(&tio, B115200);
    tio.c_cflag &= ~PARENB; tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;  tio.c_cflag |= CS8;
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    tio.c_oflag &= ~OPOST;
    tio.c_cc[VMIN] = 0; tio.c_cc[VTIME] = 10;   // timeout doc ~1.0s
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) { close(); return false; }
    return true;
}

void ZlacDriver::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool ZlacDriver::write_and_ack(const uint8_t* frame, int len, int expected_len) {
    if (fd_ < 0) return false;
    // Bo byte thua cua lan truoc, tranh doc nham lam ACK. Hang doi nhan la cua cong, dung
    // chung voi fd cua ZlacEncoder (vd tra loi FC 0x03 den tre sau khi encoder da timeout).
    tcflush(fd_, TCIFLUSH);
    ssize_t w = ::write(fd_, frame, len);
    if (w != len) {
        fprintf(stderr, "[zlac] loi write(): %s\n", strerror(errno));
        return false;
    }
    tcdrain(fd_);  

    uint8_t resp[32] = {0};
    int n = read_exact(fd_, resp, expected_len);
    if (n == 0) {
        fprintf(stderr, "[zlac] khong co ACK (timeout/mat ket noi)\n");
        return false;
    }
    if (n >= 2 && (resp[1] & 0x80)) {
        uint8_t err = (n >= 3) ? resp[2] : 0;
        fprintf(stderr, "[zlac] EXCEPTION: fc=0x%02X ma_loi=0x%02X\n", resp[1], err);
        return false;
    }
    return true;
}

bool ZlacDriver::send_acc_left(uint16_t reg, uint16_t val) {
    uint8_t f[8]; int i = 0;
    f[i++] = cfg_.slave; f[i++] = 0x06;
    f[i++] = (reg >> 8) & 0xFF; f[i++] = reg & 0xFF;
    f[i++] = (val >> 8) & 0xFF; f[i++] = val & 0xFF;
    uint16_t crc = modbus_crc(f, i);
    f[i++] = crc & 0xFF; f[i++] = (crc >> 8) & 0xFF;
    return write_and_ack(f, i, 8);   // FC 0x06: tra loi echo 8 byte
}

bool ZlacDriver::send_single(uint16_t reg, uint16_t val) {
    uint8_t f[8]; int i = 0;
    f[i++] = cfg_.slave; f[i++] = 0x06;
    f[i++] = (reg >> 8) & 0xFF; f[i++] = reg & 0xFF;
    f[i++] = (val >> 8) & 0xFF; f[i++] = val & 0xFF;
    uint16_t crc = modbus_crc(f, i);
    f[i++] = crc & 0xFF; f[i++] = (crc >> 8) & 0xFF;
    return write_and_ack(f, i, 8);   // FC 0x06: tra loi echo 8 byte
}

bool ZlacDriver::send_double(int16_t word1, int16_t word2) {
    uint8_t f[13]; int i = 0;
    f[i++] = cfg_.slave; f[i++] = 0x10;
    f[i++] = (REG_VEL_ADDR >> 8) & 0xFF; f[i++] = REG_VEL_ADDR & 0xFF;
    f[i++] = 0x00; f[i++] = 0x02;    // ghi 2 word
    f[i++] = 0x04;                   // 4 byte data
    f[i++] = (word1 >> 8) & 0xFF; f[i++] = word1 & 0xFF;   // -> 0x2088 (TRAI)
    f[i++] = (word2 >> 8) & 0xFF; f[i++] = word2 & 0xFF;   // -> 0x2089 (PHAI)
    uint16_t crc = modbus_crc(f, i);
    f[i++] = crc & 0xFF; f[i++] = (crc >> 8) & 0xFF;
    return write_and_ack(f, i, 8);   // FC 0x10: tra loi 8 byte (addr+fc+reg+qty+crc)
}

bool ZlacDriver::set_velocity_mode() { return send_single(REG_MODE,   VAL_VEL_MODE); }
bool ZlacDriver::enable()            { return send_single(REG_ENABLE, VAL_ENABLE);   }
bool ZlacDriver::disable()           { return send_single(REG_ENABLE, VAL_DISABLE);  }
bool ZlacDriver::stop()              { return send_double(0, 0); }

void ZlacDriver::inverse_kinematics(double v, double omega,
                                    double& rpm_left, double& rpm_right) const {
    double v_left  = v - omega * (cfg_.wheel_base / 2.0);
    double v_right = v + omega * (cfg_.wheel_base / 2.0);
    rpm_left  = (v_left  / cfg_.wheel_radius) * 60.0 / (2.0 * M_PI);
    rpm_right = (v_right / cfg_.wheel_radius) * 60.0 / (2.0 * M_PI);
}

bool ZlacDriver::send_velocity(double v, double omega, bool& clamped) {
    clamped = false;
    if (std::fabs(v) > cfg_.max_linear_speed) {
        return false;
    }

    double rpm_L, rpm_R;
    inverse_kinematics(v, omega, rpm_L, rpm_R);

    double cmd_L = rpm_L * cfg_.dir_left;
    double cmd_R = rpm_R * cfg_.dir_right;
    double sat_L = std::clamp(cmd_L, -cfg_.rpm_max, cfg_.rpm_max);
    double sat_R = std::clamp(cmd_R, -cfg_.rpm_max, cfg_.rpm_max);
    clamped = (sat_L != cmd_L) || (sat_R != cmd_R);

    int16_t word1 = static_cast<int16_t>(std::lround(sat_L));   // TRAI
    int16_t word2 = static_cast<int16_t>(std::lround(sat_R));   // PHAI
    return send_double(word1, word2);
}