// zlac_encoder.hpp — doc encoder 2 banh tu driver ZLAC8015D (Modbus RTU, FC 0x03).
// Khong phu thuoc ROS, khong phu thuoc class ZlacDriver.
// Dung chung cong /dev/ttyUSB_driver voi ZlacDriver (moi ben mo fd rieng). KHONG khoa cong:
// ZlacDriver::open() da giu khoa doc quyen, khoa them o day se bi chinh khoa do chan.
// Hai ben phai duoc goi tu cung 1 luong (zlac_node dung executor don luong) de khong hoi-dap chong nhau.

#pragma once
#include <cstdint>

struct ZlacEncoderConfig {
    // Serial (cung cong voi ZlacDriver)
    const char* device   = "/dev/ttyUSB_driver";
    uint8_t     slave    = 0x01;

    // Encoder vuong pha: 1 vach = 4 counts (so vach doc tu driver, 0x2030/0x2060)
    int    counts_per_line = 4;

    // Chieu lap banh — PHAI KHOP voi ZlacConfig (zlac_node.cpp gan lai tu ZlacConfig).
    // Nhan vao counts/rpm tho de xe tien thi ca 2 banh deu duong.
    double dir_left      = +1.0;
    double dir_right     = -1.0;
};

// Du lieu tho doc ve tu driver, CHUA nhan dir_left/dir_right
struct ZlacFeedback {
    int32_t counts_left  = 0;     // vi tri cong don tu luc driver bat nguon (counts)
    int32_t counts_right = 0;
    double  rpm_left     = 0.0;   // toc do thuc (r/min)
    double  rpm_right    = 0.0;
};

class ZlacEncoder {
public:
    explicit ZlacEncoder(const ZlacEncoderConfig& cfg = ZlacEncoderConfig());
    ~ZlacEncoder();

    bool open();
    void close();
    bool is_open() const { return fd_ >= 0; }

    bool read_encoder_lines(uint16_t& left, uint16_t& right);   // so vach, 0x2030 / 0x2060
    bool read_feedback(ZlacFeedback& fb);                       // vi tri + toc do 2 banh, 1 frame

    const ZlacEncoderConfig& config() const { return cfg_; }

private:
    bool read_registers(uint16_t reg, uint16_t count, uint16_t* out);   // FC 0x03

    ZlacEncoderConfig cfg_;
    int                fd_ = -1;
};
