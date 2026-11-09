// zlac_node.cpp — 1 tien trinh, 2 node dung chung driver ZLAC8015D:
//   node "zlac_driver" : nhan /cmd_vel -> dieu khien banh.
//   node "zlac_encoder": doc encoder 50 Hz -> /encoder (chi thu du lieu, khong tinh odom).
//                        Tham so enabled (mac dinh true): false = ngung doc + ngung phat.
//                          luc khoi dong: --ros-args -p zlac_encoder:enabled:=false
//                          dang chay    : ros2 param set /zlac_encoder enabled false|true
// Hai node chay chung 1 executor don luong nen callback /cmd_vel va timer encoder chay
// lan luot, khong bao gio hoi-dap chong nhau tren /dev/ttyUSB_driver.
// ZlacDriver::open() khoa doc quyen cong (chan zlac_node thu 2); ZlacEncoder mo fd rieng va
// khong khoa, nen doi sang MultiThreadedExecutor hay them luong dung cong thi phai tu them
// khoa (vd std::mutex) quanh moi lan hoi-dap.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "zlac_driver/zlac_driver.hpp"
#include "zlac_driver/zlac_encoder.hpp"

class ZlacDriverNode : public rclcpp::Node {
public:
  explicit ZlacDriverNode(const ZlacConfig& cfg) : Node("zlac_driver"), drv_(cfg) {
    if (!drv_.open() || !drv_.set_velocity_mode() || !drv_.enable()) {
      throw std::runtime_error("khong khoi tao duoc ZLAC (mo cong / velocity mode / enable)");
    }
    drv_.send_acc_left(REG_ACC_ADDR1, 500);
    drv_.send_acc_left(REG_ACC_ADDR2, 500);
    drv_.send_acc_left(REG_DEC_ADDR1, 500);
    drv_.send_acc_left(REG_DEC_ADDR2, 500);
    drv_.send_acc_left(REG_DEC_ADDR2, 500);

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          this->onCmdVel_callback(msg);
        });

    RCLCPP_INFO(get_logger(), "zlac_driver san sang, dang nghe /cmd_vel");
  }

  ~ZlacDriverNode() {
    drv_.stop();
    drv_.disable();
  }

private:
  void onCmdVel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double v = msg->linear.x;
    double w = msg->angular.z;
    bool clamped = false;
    drv_.send_velocity(v, w, clamped);
    if (clamped)
      RCLCPP_WARN(get_logger(), "lenh vuot rpm_max, da bi cat");
  }

  ZlacDriver drv_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

// Chi DOC tu driver, khong bao gio ghi toc do; dieu khien banh la viec cua ZlacDriverNode.
class EncoderNode : public rclcpp::Node {
public:
  explicit EncoderNode(ZlacEncoder& enc) : Node("zlac_encoder"), enc_(enc) {
    uint16_t lines_l = 0, lines_r = 0;
    if (!enc_.read_encoder_lines(lines_l, lines_r) || lines_l == 0 || lines_r == 0) {
      throw std::runtime_error("khong doc duoc so vach encoder (0x2030/0x2060)");
    }
    // Nhan san dir: counts/rpm tho -> goc/van toc goc banh, duong = xe tien
    const ZlacEncoderConfig& cfg = enc_.config();
    rad_per_count_l_ = cfg.dir_left  * 2.0 * M_PI / (lines_l * cfg.counts_per_line);
    rad_per_count_r_ = cfg.dir_right * 2.0 * M_PI / (lines_r * cfg.counts_per_line);
    radps_per_rpm_l_ = cfg.dir_left  * 2.0 * M_PI / 60.0;
    radps_per_rpm_r_ = cfg.dir_right * 2.0 * M_PI / 60.0;

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/encoder", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               [this]() { this->onTimer_callback(); });

    // Tat = dung timer: khong gui FC 0x03, khong phat /encoder. Bo dem counts trong driver
    // van dem, nen bat lai thi lan doc dau cong ca quang banh quay trong luc tat.
    const bool enabled = declare_parameter("enabled", true);
    if (!enabled) timer_->cancel();
    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
          return this->onSetParam_callback(params);
        });

    RCLCPP_INFO(get_logger(), "zlac_encoder san sang, %s (encoder %u/%u vach)",
                enabled ? "phat /encoder" : "dang TAT (enabled=false)", lines_l, lines_r);
  }

private:
  // Kieu bool da duoc rclcpp kiem truoc khi goi callback nay (khai bao mac dinh true)
  rcl_interfaces::msg::SetParametersResult onSetParam_callback(
      const std::vector<rclcpp::Parameter>& params) {
    for (const auto& p : params) {
      if (p.get_name() != "enabled") continue;
      if (p.as_bool()) {
        timer_->reset();
        RCLCPP_INFO(get_logger(), "BAT: doc encoder, phat /encoder");
      } else {
        timer_->cancel();
        RCLCPP_INFO(get_logger(), "TAT: ngung doc encoder, ngung phat /encoder");
      }
    }
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    return res;
  }

  void onTimer_callback() {
    ZlacFeedback fb;
    if (!enc_.read_feedback(fb)) return;
    const rclcpp::Time stamp = now();

    if (!have_prev_) {   // mau dau tien chi lam moc: goc banh bat dau tu 0
      prev_ = fb;
      have_prev_ = true;
    }
    // Tru tren uint32 roi ep ve int32: van dung khi bo dem counts tran
    total_l_ += static_cast<int32_t>(
        static_cast<uint32_t>(fb.counts_left) - static_cast<uint32_t>(prev_.counts_left));
    total_r_ += static_cast<int32_t>(
        static_cast<uint32_t>(fb.counts_right) - static_cast<uint32_t>(prev_.counts_right));
    prev_ = fb;

    // goc (rad) va van toc goc (rad/s) cua tung banh
    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name     = {"left_wheel_joint", "right_wheel_joint"};
    js.position = {total_l_ * rad_per_count_l_, total_r_ * rad_per_count_r_};
    js.velocity = {fb.rpm_left * radps_per_rpm_l_, fb.rpm_right * radps_per_rpm_r_};
    joint_pub_->publish(js);
  }

  ZlacEncoder& enc_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  double rad_per_count_l_ = 0.0, rad_per_count_r_ = 0.0;   // rad / count, da nhan dir
  double radps_per_rpm_l_ = 0.0, radps_per_rpm_r_ = 0.0;   // (rad/s) / (r/min), da nhan dir

  ZlacFeedback prev_;
  bool    have_prev_ = false;
  int64_t total_l_ = 0, total_r_ = 0;   // counts cong don tu luc node khoi dong
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    // Dung node driver TRUOC: zlac_node thu 2 bi tu choi o day (cong da bi khoa) va thoat
    // truoc khi mo encoder, khong gui frame nao.
    ZlacConfig cfg;
    auto driver_node = std::make_shared<ZlacDriverNode>(cfg);

    // Cong, slave va chieu lap banh lay tu ZlacConfig: chi can sua o zlac_driver.hpp
    ZlacEncoderConfig enc_cfg;
    enc_cfg.device    = cfg.device;
    enc_cfg.slave     = cfg.slave;
    enc_cfg.dir_left  = cfg.dir_left;
    enc_cfg.dir_right = cfg.dir_right;
    ZlacEncoder enc(enc_cfg);

    // Encoder loi thi chi bao loi, node zlac_driver van chay de dieu khien xe
    std::shared_ptr<EncoderNode> encoder_node;
    try {
      if (!enc.open()) {
        throw std::runtime_error("khong mo duoc cong driver");
      }
      encoder_node = std::make_shared<EncoderNode>(enc);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("zlac_encoder"), "%s -> khong phat /encoder", e.what());
      enc.close();
    }

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(driver_node);
    if (encoder_node) exec.add_node(encoder_node);
    exec.spin();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("zlac_driver"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
