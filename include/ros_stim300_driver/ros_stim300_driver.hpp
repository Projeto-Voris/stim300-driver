#ifndef ROS_STIM300_DRIVER_ROS_STIM300_DRIVER_HPP
#define ROS_STIM300_DRIVER_ROS_STIM300_DRIVER_HPP

#include "iostream"
#include "math.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "stim300_driver/driver_stim300.hpp"
#include "stim300_driver/serial_unix.hpp"
#include <vector>

constexpr int NUMBER_OF_CALIBRATION_SAMPLES{100};
constexpr double ACC_TOLERANCE{0.1};
constexpr double MAX_DROPPED_ACC_X_MSG{5};
constexpr double MAX_DROPPED_ACC_Y_MSG{5};
constexpr double MAX_DROPPED_ACC_Z_MSG{5};
constexpr double MAX_DROPPED_GYRO_X_MSG{5};
constexpr double MAX_DROPPED_GYRO_Y_MSG{5};
constexpr double MAX_DROPPED_GYRO_Z_MSG{5};
constexpr double GYRO_X_PEAK_TO_PEAK_NOISE{0.0002};
constexpr double GYRO_Y_PEAK_TO_PEAK_NOISE{0.0002};
constexpr double GYRO_Z_PEAK_TO_PEAK_NOISE{0.0002};

struct Quaternion {
  double w, x, y, z;
};

struct EulerAngles {
  double roll, pitch, yaw;
};

Quaternion
FromRPYToQuaternion(EulerAngles angles); // yaw (Z), pitch (Y), roll (X)

class Stim300DriverNode : public rclcpp::Node {
public:
  Stim300DriverNode();

private:
  void timerCallback();

  bool responseCalibrateIMU(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool calibration_mode_ = false;

  std::string device_name_;
  double variance_gyro_;
  double variance_acc_;
  double gravity_;
  double sample_rate_;

  sensor_msgs::msg::Imu stim300msg_;

  std::unique_ptr<SerialUnix> serial_driver_;
  std::unique_ptr<DriverStim300> driver_stim300_;
};

#endif // ROS_STIM300_DRIVER_ROS_STIM300_DRIVER_HPP