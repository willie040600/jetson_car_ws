// YDLidar SDK -> sensor_msgs/LaserScan (/scan)
//
// initialize() 連線並校驗參數，turnOn() 啟動掃描執行緒與雷達馬達，
// doProcessSimple() 阻塞取得一整圈資料後轉成 LaserScan 發佈。

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>

#include "CYdLidar.h"

class YdlidarScanNode : public rclcpp::Node
{
public:
  YdlidarScanNode()
  : Node("ydlidar_scan_node")
  {
    const auto port = declare_parameter<std::string>("port", "/dev/ydlidar");
    const auto ignore_array = declare_parameter<std::string>("ignore_array", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "laser_frame");

    const int baudrate = declare_parameter<int>("baudrate", 230400);
    const int lidar_type = declare_parameter<int>("lidar_type", TYPE_TRIANGLE);
    const int device_type = declare_parameter<int>("device_type", YDLIDAR_TYPE_SERIAL);
    const int sample_rate = declare_parameter<int>("sample_rate", 9);
    const int abnormal_check_count = declare_parameter<int>("abnormal_check_count", 4);
    const int intensity_bit = declare_parameter<int>("intensity_bit", 0);

    const auto fixed_resolution = declare_parameter<bool>("fixed_resolution", true);
    const auto reversion = declare_parameter<bool>("reversion", false);
    const auto inverted = declare_parameter<bool>("inverted", false);
    const auto auto_reconnect = declare_parameter<bool>("auto_reconnect", true);
    const auto single_channel = declare_parameter<bool>("isSingleChannel", false);
    const auto intensity = declare_parameter<bool>("intensity", false);
    const auto support_motor_dtr = declare_parameter<bool>("support_motor_dtr", false);

    const auto angle_max = declare_parameter<double>("angle_max", 180.0);
    const auto angle_min = declare_parameter<double>("angle_min", -180.0);
    const auto range_max = declare_parameter<double>("range_max", 12.0);
    const auto range_min = declare_parameter<double>("range_min", 0.1);
    const auto frequency = declare_parameter<double>("frequency", 10.0);
    invalid_range_is_inf_ = declare_parameter<bool>("invalid_range_is_inf", false);

    laser_.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
    laser_.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(), ignore_array.size());

    setInt(LidarPropSerialBaudrate, baudrate);
    setInt(LidarPropLidarType, lidar_type);
    setInt(LidarPropDeviceType, device_type);
    setInt(LidarPropSampleRate, sample_rate);
    setInt(LidarPropAbnormalCheckCount, abnormal_check_count);
    setInt(LidarPropIntenstiyBit, intensity_bit);

    setBool(LidarPropFixedResolution, fixed_resolution);
    setBool(LidarPropReversion, reversion);
    setBool(LidarPropInverted, inverted);
    setBool(LidarPropAutoReconnect, auto_reconnect);
    setBool(LidarPropSingleChannel, single_channel);
    setBool(LidarPropIntenstiy, intensity);
    setBool(LidarPropSupportMotorDtrCtrl, support_motor_dtr);

    setFloat(LidarPropMaxAngle, angle_max);
    setFloat(LidarPropMinAngle, angle_min);
    setFloat(LidarPropMaxRange, range_max);
    setFloat(LidarPropMinRange, range_min);
    setFloat(LidarPropScanFrequency, frequency);

    if (!laser_.initialize()) {
      throw std::runtime_error(std::string("YDLidar initialize failed: ") + laser_.DescribeError());
    }
    if (!laser_.turnOn()) {
      laser_.disconnecting();
      throw std::runtime_error(std::string("YDLidar turnOn failed: ") + laser_.DescribeError());
    }
    connected_ = true;
    RCLCPP_INFO(get_logger(), "YDLidar started on %s @ %d bps", port.c_str(), baudrate);

    // 用 reliable（非 SensorDataQoS），才能同時相容 best-effort 與 reliable 的訂閱者（如 slam_toolbox）。
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::KeepLast(10)));

    start_srv_ = create_service<std_srvs::srv::Empty>(
      "start_scan",
      [this](
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>) {
        scanning_ = laser_.turnOn();
        RCLCPP_INFO(get_logger(), "start_scan -> %s", scanning_ ? "on" : "failed");
      });
    stop_srv_ = create_service<std_srvs::srv::Empty>(
      "stop_scan",
      [this](
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>) {
        laser_.turnOff();
        scanning_ = false;
        RCLCPP_INFO(get_logger(), "stop_scan -> off");
      });

    // doProcessSimple() 會阻塞等下一圈，因此放在獨立執行緒而非 timer，避免卡住 service callback。
    scanning_ = true;
    worker_ = std::thread(&YdlidarScanNode::spinLidar, this);
  }

  ~YdlidarScanNode() override
  {
    running_ = false;
    if (worker_.joinable()) {
      worker_.join();
    }
    if (connected_) {
      laser_.turnOff();
      laser_.disconnecting();
    }
  }

private:
  void setInt(LidarProperty prop, int value)
  {
    laser_.setlidaropt(prop, &value, sizeof(int));
  }

  void setBool(LidarProperty prop, bool value)
  {
    laser_.setlidaropt(prop, &value, sizeof(bool));
  }

  void setFloat(LidarProperty prop, double value)
  {
    auto v = static_cast<float>(value);
    laser_.setlidaropt(prop, &v, sizeof(float));
  }

  void spinLidar()
  {
    while (running_ && rclcpp::ok()) {
      if (!scanning_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      LaserScan scan;
      if (!laser_.doProcessSimple(scan)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to get Lidar data");
        continue;
      }
      publishScan(scan);
    }
  }

  void publishScan(const LaserScan & scan)
  {
    if (scan.config.angle_increment <= 0.0f) {
      return;
    }
    const int size = static_cast<int>(
      (scan.config.max_angle - scan.config.min_angle) / scan.config.angle_increment) + 1;
    if (size <= 0) {
      return;
    }

    auto msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    // SDK 的 scan.stamp 與 ROS 時鐘不同源，會讓 tf2 查不到對應時刻的 odom；改用掃描起始的 ROS 時間。
    msg->header.stamp = now() - rclcpp::Duration::from_seconds(scan.config.scan_time);
    msg->header.frame_id = frame_id_;
    msg->angle_min = scan.config.min_angle;
    msg->angle_max = scan.config.max_angle;
    msg->angle_increment = scan.config.angle_increment;
    msg->scan_time = scan.config.scan_time;
    msg->time_increment = scan.config.time_increment;
    msg->range_min = scan.config.min_range;
    msg->range_max = scan.config.max_range;

    const float invalid = invalid_range_is_inf_
      ? std::numeric_limits<float>::infinity()
      : 0.0f;
    msg->ranges.assign(static_cast<size_t>(size), invalid);
    msg->intensities.assign(static_cast<size_t>(size), 0.0f);

    for (const auto & p : scan.points) {
      const int index = static_cast<int>(
        std::ceil((p.angle - scan.config.min_angle) / scan.config.angle_increment));
      if (index < 0 || index >= size) {
        continue;
      }
      msg->ranges[static_cast<size_t>(index)] =
        (p.range >= scan.config.min_range && p.range <= scan.config.max_range) ? p.range : invalid;
      msg->intensities[static_cast<size_t>(index)] = p.intensity;
    }

    pub_->publish(std::move(msg));
  }

  CYdLidar laser_;
  std::string frame_id_;
  bool invalid_range_is_inf_{false};
  bool connected_{false};
  std::atomic<bool> running_{true};
  std::atomic<bool> scanning_{false};
  std::thread worker_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<YdlidarScanNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("ydlidar_scan_node"), "%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
