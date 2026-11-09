// zlac_encoder.cpp — implementation cua ZlacEncoder.

#include "zlac_driver/zlac_encoder.hpp"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <cstdio>

namespace {
constexpr uint16_t REG_ENC_LINE_L = 0x2030;
constexpr uint16_t REG_ENC_LINE_R = 0x2060;
// 6 word lien tiep: vi tri TRAI (hi, lo), vi tri PHAI (hi, lo), toc do TRAI, toc do PHAI
constexpr uint16_t REG_FEEDBACK   = 0x20A7;
constexpr int      MAX_READ_REGS  = 16;

// modbus_crc va read_exact chep tu zlac_driver.cpp (giu nguyen file thu vien cu)
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

ZlacEncoder::ZlacEncoder(const ZlacEncoderConfig& cfg) : cfg_(cfg) {}

ZlacEncoder::~ZlacEncoder() { close(); }

bool ZlacEncoder::open() {
    fd_ = ::open(cfg_.device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        fprintf(stderr, "[zlac_encoder] khong mo duoc %s: %s\n", cfg_.device, strerror(errno));
        return false;
    }
    // Cau hinh y het ZlacDriver::open(): cai dat termios thuoc ve cong, nen
    // hai class cung mo /dev/ttyUSB_driver thi phai giong nhau.
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

void ZlacEncoder::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool ZlacEncoder::read_registers(uint16_t reg, uint16_t count, uint16_t* out) {
    if (fd_ < 0 || count == 0 || count > MAX_READ_REGS) return false;
    uint8_t f[8]; int i = 0;
    f[i++] = cfg_.slave; f[i++] = 0x03;
    f[i++] = (reg >> 8) & 0xFF; f[i++] = reg & 0xFF;
    f[i++] = (count >> 8) & 0xFF; f[i++] = count & 0xFF;
    uint16_t crc = modbus_crc(f, i);
    f[i++] = crc & 0xFF; f[i++] = (crc >> 8) & 0xFF;

    tcflush(fd_, TCIFLUSH);   // bo byte thua cua lan truoc, tranh lech frame
    if (::write(fd_, f, i) != i) {
        fprintf(stderr, "[zlac_encoder] loi write(): %s\n", strerror(errno));
        return false;
    }
    tcdrain(fd_);

    // FC 0x03 tra loi: addr + fc + so_byte + 2*count byte du lieu + crc
    uint8_t resp[5 + 2 * MAX_READ_REGS] = {0};
    const int want = 5 + 2 * count;
    int n = read_exact(fd_, resp, want);
    if (n == 0) {
        fprintf(stderr, "[zlac_encoder] doc 0x%04X: khong co tra loi (timeout/mat ket noi)\n", reg);
        return false;
    }
    if (n >= 2 && (resp[1] & 0x80)) {
        uint8_t err = (n >= 3) ? resp[2] : 0;
        fprintf(stderr, "[zlac_encoder] EXCEPTION: fc=0x%02X ma_loi=0x%02X\n", resp[1], err);
        return false;
    }
    // Du lieu doc ve duoc dem di tinh toan nen phai kiem du frame + CRC
    if (n != want || resp[0] != cfg_.slave || resp[1] != 0x03 || resp[2] != 2 * count ||
        modbus_crc(resp, want - 2) != (resp[want - 2] | (resp[want - 1] << 8))) {
        fprintf(stderr, "[zlac_encoder] doc 0x%04X: frame tra loi hong (%d/%d byte)\n", reg, n, want);
        return false;
    }
    for (int k = 0; k < count; k++)
        out[k] = (resp[3 + 2 * k] << 8) | resp[4 + 2 * k];
    return true;
}

bool ZlacEncoder::read_feedback(ZlacFeedback& fb) {
    uint16_t w[6];
    if (!read_registers(REG_FEEDBACK, 6, w)) return false;
    fb.counts_left  = static_cast<int32_t>((static_cast<uint32_t>(w[0]) << 16) | w[1]);
    fb.counts_right = static_cast<int32_t>((static_cast<uint32_t>(w[2]) << 16) | w[3]);
    fb.rpm_left     = static_cast<int16_t>(w[4]) * 0.1;   // thanh ghi tinh theo 0.1 r/min
    fb.rpm_right    = static_cast<int16_t>(w[5]) * 0.1;
    return true;
}

bool ZlacEncoder::read_encoder_lines(uint16_t& left, uint16_t& right) {
    return read_registers(REG_ENC_LINE_L, 1, &left) &&
           read_registers(REG_ENC_LINE_R, 1, &right);
}
