// ZLAC706-RC 差速輪 ROS 2 節點 (rclcpp)
//
// Modbus RTU 手動組封包 + CRC16，協定對應已驗證可動的測試腳本：
//   - 實接線：左輪 slave address = 0x02, 右輪 slave address = 0x01
//   - 寫入速度命令：暫存器 0x0011 (function code 0x06)
//   - 讀取狀態+速度：暫存器 0x00D2, 長度 2 (function code 0x03)
//   - 使能指令：對 0x0010 寫入固定值（每輪各自的 CRC 已內嵌）
//   - RPM <-> 內部整數換算：data = rpm / 3000 * 8192
//
// 所有序列埠存取集中在單一 timer callback，cmd_vel 僅更新目標值，避免搶匯流排。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace
{

// 實測左右輪接在與直覺相反的 slave address，接反時只有轉向會錯、直行看不出來。
constexpr uint8_t ADDR_L = 0x02;
constexpr uint8_t ADDR_R = 0x01;

// 驅動器實測需要每幀間隔 >=30ms 才會回應，間隔由 timer 週期提供，不在 callback 內 sleep。
constexpr size_t WRITE_REPLY_LEN = 8;
constexpr size_t READ_REPLY_LEN = 9;

// 使能命令（沿用已驗證可動的固定 bytes，內含各自的 CRC）——依 address 命名，與左右無關
const std::vector<uint8_t> ENABLE_ADDR_01{0x01, 0x06, 0x00, 0x10, 0x00, 0x1F, 0xC9, 0xC7};
const std::vector<uint8_t> ENABLE_ADDR_02{0x02, 0x06, 0x00, 0x10, 0x00, 0x1F, 0xC9, 0xF4};

uint16_t modbusCrc(const std::vector<uint8_t> & data)
{
  uint16_t crc = 0xFFFF;
  for (uint8_t b : data) {
    crc ^= b;
    for (int i = 0; i < 8; ++i) {
      crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
    }
  }
  return crc;
}

std::vector<uint8_t> appendCrc(std::vector<uint8_t> payload)
{
  const uint16_t crc = modbusCrc(payload);
  payload.push_back(static_cast<uint8_t>(crc & 0xFF));
  payload.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  return payload;
}

std::vector<uint8_t> buildSpeedCmd(uint8_t addr, double rpm)
{
  const auto value = static_cast<uint16_t>(static_cast<int32_t>(rpm / 3000.0 * 8192.0) & 0xFFFF);
  return appendCrc({addr, 0x06, 0x00, 0x11,
      static_cast<uint8_t>((value >> 8) & 0xFF), static_cast<uint8_t>(value & 0xFF)});
}

std::vector<uint8_t> readStrpmCmd(uint8_t addr)
{
  return appendCrc({addr, 0x03, 0x00, 0xD2, 0x00, 0x02});
}

/// 在位元組流中找出 CRC 正確的目標幀，藉此跳過 RS-485 回音與前導雜訊。
std::optional<std::vector<uint8_t>> findFrame(
  const std::vector<uint8_t> & buffer, uint8_t addr, uint8_t func, size_t len)
{
  for (size_t i = 0; i + len <= buffer.size(); ++i) {
    if (buffer[i] != addr || buffer[i + 1] != func) {
      continue;
    }
    const std::vector<uint8_t> body(buffer.begin() + i, buffer.begin() + i + len - 2);
    const uint16_t crc = modbusCrc(body);
    if ((crc & 0xFF) == buffer[i + len - 2] && ((crc >> 8) & 0xFF) == buffer[i + len - 1]) {
      return std::vector<uint8_t>(buffer.begin() + i, buffer.begin() + i + len);
    }
  }
  return std::nullopt;
}

std::optional<int16_t> parseSpeedReply(const std::vector<uint8_t> & frame)
{
  if (frame.size() < READ_REPLY_LEN || frame[2] != 0x04) {
    return std::nullopt;
  }
  return static_cast<int16_t>((frame[5] << 8) | frame[6]);
}

std::string toHex(const std::vector<uint8_t> & data)
{
  static const char * kDigits = "0123456789ABCDEF";
  std::string out;
  out.reserve(data.size() * 3);
  for (uint8_t b : data) {
    out.push_back(kDigits[b >> 4]);
    out.push_back(kDigits[b & 0x0F]);
    out.push_back(' ');
  }
  return out;
}

double rpmFromRaw(int16_t raw)
{
  return raw * 3000.0 / 8192.0;
}

std::optional<speed_t> toBaudConstant(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return std::nullopt;
  }
}

/// 以 termios 操作的 RS-485 序列埠，每次交易 = 送出請求 → 收到 CRC 正確的目標幀。
class SerialPort
{
public:
  ~SerialPort() {close();}

  bool open(const std::string & device, int baud, double reply_timeout, std::string & error)
  {
    reply_timeout_ = reply_timeout;
    const auto speed = toBaudConstant(baud);
    if (!speed) {
      error = "不支援的 baudrate: " + std::to_string(baud);
      return false;
    }

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      error = device + ": " + std::strerror(errno);
      return false;
    }

    termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) {
      error = std::strerror(errno);
      close();
      return false;
    }

    ::cfmakeraw(&tty);
    ::cfsetispeed(&tty, *speed);
    ::cfsetospeed(&tty, *speed);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~static_cast<tcflag_t>(CSTOPB);   // 1 stop bit
    tty.c_cflag &= ~static_cast<tcflag_t>(PARENB);   // no parity
    tty.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);  // no hardware flow control
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
      error = std::strerror(errno);
      close();
      return false;
    }
    ::tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool isOpen() const {return fd_ >= 0;}

  std::vector<uint8_t> transact(
    const std::vector<uint8_t> & request, uint8_t expect_addr, uint8_t expect_func,
    size_t expect_len, std::vector<uint8_t> * raw_out = nullptr)
  {
    std::vector<uint8_t> buffer;
    if (fd_ < 0) {
      return {};
    }

    ::tcflush(fd_, TCIFLUSH);
    if (::write(fd_, request.data(), request.size()) < 0) {
      return {};
    }
    ::tcdrain(fd_);

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(reply_timeout_));

    uint8_t chunk[64];
    std::optional<std::vector<uint8_t>> frame;
    while (!frame) {
      const auto remaining = deadline - std::chrono::steady_clock::now();
      if (remaining <= std::chrono::steady_clock::duration::zero()) {
        break;
      }
      const auto usec = std::chrono::duration_cast<std::chrono::microseconds>(remaining).count();

      timeval tv{static_cast<time_t>(usec / 1000000), static_cast<suseconds_t>(usec % 1000000)};
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(fd_, &rfds);
      if (::select(fd_ + 1, &rfds, nullptr, nullptr, &tv) <= 0) {
        break;
      }

      const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
      if (n <= 0) {
        break;
      }
      buffer.insert(buffer.end(), chunk, chunk + n);
      frame = findFrame(buffer, expect_addr, expect_func, expect_len);
    }

    if (raw_out != nullptr) {
      *raw_out = buffer;
    }
    return frame ? *frame : std::vector<uint8_t>{};
  }

private:
  int fd_{-1};
  double reply_timeout_{0.05};
};

}  // namespace

class Zlac706DiffDrive : public rclcpp::Node
{
public:
  Zlac706DiffDrive()
  : Node("zlac706_diffdrive_node")
  {
    const auto port = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    const int baud = static_cast<int>(declare_parameter<int64_t>("baudrate", 38400));
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.0508);
    wheel_sep_ = declare_parameter<double>("wheel_separation", 0.23);
    max_rpm_ = declare_parameter<double>("max_rpm", 200.0);
    // 驅動器需要幀間 >=30ms，再加上回應延遲，50ms 一筆交易才穩
    const auto transaction_period = declare_parameter<double>("transaction_period", 0.03);
    invert_left_ = declare_parameter<bool>("invert_left", false);
    invert_right_ = declare_parameter<bool>("invert_right", true);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);
    const auto reply_timeout = declare_parameter<double>("reply_timeout", 0.05);
    debug_serial_ = declare_parameter<bool>("debug_serial", false);

    std::string error;
    if (serial_.open(port, baud, reply_timeout, error)) {
      RCLCPP_INFO(get_logger(), "序列埠已開啟: %s @ %dbps", port.c_str(), baud);
      enableDriver();
    } else {
      RCLCPP_ERROR(get_logger(), "無法開啟序列埠 %s", error.c_str());
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&Zlac706DiffDrive::cmdVelCb, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_time_ = now();
    last_cmd_time_ = std::chrono::steady_clock::now();

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(transaction_period)),
      std::bind(&Zlac706DiffDrive::stepCb, this));
  }

  ~Zlac706DiffDrive() override
  {
    writeWheel(ADDR_L, 0.0, invert_left_);
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    writeWheel(ADDR_R, 0.0, invert_right_);
    serial_.close();
  }

private:
  void enableDriver()
  {
    serial_.transact(ENABLE_ADDR_01, 0x01, 0x06, WRITE_REPLY_LEN);
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    serial_.transact(ENABLE_ADDR_02, 0x02, 0x06, WRITE_REPLY_LEN);
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
  }

  void writeWheel(uint8_t addr, double rpm, bool invert)
  {
    if (!serial_.isOpen()) {
      return;
    }
    rpm = std::clamp(rpm, -max_rpm_, max_rpm_);
    if (invert) {
      rpm = -rpm;
    }
    serial_.transact(buildSpeedCmd(addr, rpm), addr, 0x06, WRITE_REPLY_LEN);
  }

  std::optional<double> readWheel(uint8_t addr, bool invert)
  {
    if (!serial_.isOpen()) {
      return std::nullopt;
    }
    std::vector<uint8_t> raw;
    const auto rpm_raw = parseSpeedReply(
      serial_.transact(
        readStrpmCmd(addr), addr, 0x03, READ_REPLY_LEN, debug_serial_ ? &raw : nullptr));
    if (!rpm_raw) {
      if (debug_serial_) {
        RCLCPP_WARN(
          get_logger(), "讀取轉速回授失敗 addr=0x%02X | raw: %s", addr, toHex(raw).c_str());
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "讀取轉速回授失敗（加 -p debug_serial:=true 可看原始位元組）");
      }
      return std::nullopt;
    }
    const double rpm = rpmFromRaw(*rpm_raw);
    return invert ? -rpm : rpm;
  }

  void cmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = std::chrono::steady_clock::now();
    const double v = msg->linear.x;
    const double w = msg->angular.z;

    const double v_left = v - (w * wheel_sep_ / 2.0);
    const double v_right = v + (w * wheel_sep_ / 2.0);

    target_rpm_l_ = (v_left / (2.0 * M_PI * wheel_radius_)) * 60.0;
    target_rpm_r_ = (v_right / (2.0 * M_PI * wheel_radius_)) * 60.0;
  }

  /// 一個 tick 只做一筆交易，timer 週期就是驅動器需要的幀間間隔。
  void stepCb()
  {
    switch (step_) {
      case 0:
        {
          const std::chrono::duration<double> since_cmd =
            std::chrono::steady_clock::now() - last_cmd_time_;
          if (since_cmd.count() > cmd_timeout_) {
            target_rpm_l_ = 0.0;
            target_rpm_r_ = 0.0;
          }
          writeWheel(ADDR_L, target_rpm_l_, invert_left_);
          break;
        }
      case 1:
        writeWheel(ADDR_R, target_rpm_r_, invert_right_);
        break;
      case 2:
        fb_rpm_l_ = readWheel(ADDR_L, invert_left_);
        break;
      default:
        fb_rpm_r_ = readWheel(ADDR_R, invert_right_);
        break;
    }
    // 每個 tick 都積分，TF 才有 20Hz；只在第 4 步發會掉到 5Hz，SLAM 會查不到 odom。
    updateOdom();
    step_ = (step_ + 1) % 4;
  }

  void updateOdom()
  {
    const rclcpp::Time stamp = now();
    const double dt = (stamp - last_time_).seconds();
    last_time_ = stamp;
    if (dt <= 0.0) {
      return;
    }

    // 回授缺任一輪就當靜止，但仍要發 TF，否則 odom->base_link 斷掉會讓 SLAM/Nav2 全掛。
    double v = 0.0;
    double w = 0.0;
    if (fb_rpm_l_ && fb_rpm_r_) {
      const double v_left = (*fb_rpm_l_ / 60.0) * 2.0 * M_PI * wheel_radius_;
      const double v_right = (*fb_rpm_r_ / 60.0) * 2.0 * M_PI * wheel_radius_;
      v = (v_left + v_right) / 2.0;
      w = (v_right - v_left) / wheel_sep_;
    }

    x_ += v * std::cos(theta_) * dt;
    y_ += v * std::sin(theta_) * dt;
    theta_ += w * dt;

    const double quat_z = std::sin(theta_ / 2.0);
    const double quat_w = std::cos(theta_ / 2.0);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = quat_z;
    odom.pose.pose.orientation.w = quat_w;
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.rotation.z = quat_z;
    tf.transform.rotation.w = quat_w;
    tf_broadcaster_->sendTransform(tf);
  }

  SerialPort serial_;

  double wheel_radius_{};
  double wheel_sep_{};
  double max_rpm_{};
  double cmd_timeout_{};
  bool invert_left_{};
  bool invert_right_{};
  bool debug_serial_{false};

  double target_rpm_l_{0.0};
  double target_rpm_r_{0.0};
  std::optional<double> fb_rpm_l_;
  std::optional<double> fb_rpm_r_;
  int step_{0};
  std::chrono::steady_clock::time_point last_cmd_time_;

  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};
  rclcpp::Time last_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Zlac706DiffDrive>());
  rclcpp::shutdown();
  return 0;
}
