
#pragma once
#include <cstdint>

constexpr uint16_t REG_ACC_ADDR1 = 0x2080;
constexpr uint16_t REG_ACC_ADDR2 = 0x2081;
constexpr uint16_t REG_DEC_ADDR1 = 0x2082;
constexpr uint16_t REG_DEC_ADDR2 = 0x2083;

struct ZlacConfig {
    // Serial
    const char* device   = "/dev/ttyUSB_driver";
    uint8_t     slave    = 0x01;

    double wheel_base    = 0.40;    // m, khoang cach 2 banh
    double wheel_radius  = 0.0825;  // m

    double rpm_max       = 300.0;
    // Quy uoc dau
    double dir_left      = +1.0;
    double dir_right     = -1.0;
    double max_linear_speed = 3.0;   // m/s
};

class ZlacDriver {
public:
    explicit ZlacDriver(const ZlacConfig& cfg = ZlacConfig());
    ~ZlacDriver();

    bool open();
    bool set_velocity_mode();
    bool enable();
    bool disable();
    void close();

    bool send_velocity(double v, double omega, bool& clamped);

    bool stop();

    bool is_open() const { return fd_ >= 0; }

    void inverse_kinematics(double v, double omega,
                            double& rpm_left, double& rpm_right) const;
    bool send_acc_left(uint16_t reg, uint16_t val);

private:
    bool send_single(uint16_t reg, uint16_t val);          // FC 0x06
    bool send_double(int16_t word1, int16_t word2);        // FC 0x10
    bool write_and_ack(const uint8_t* frame, int len, int expected_len);

    ZlacConfig cfg_;
    int        fd_ = -1;
};