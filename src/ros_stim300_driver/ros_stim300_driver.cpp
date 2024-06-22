#include "ros_stim300_driver/ros_stim300_driver.hpp"


Quaternion FromRPYToQuaternion(EulerAngles angles)
{
    double cy = cos(angles.yaw * 0.5);
    double sy = sin(angles.yaw * 0.5);
    double cp = cos(angles.pitch * 0.5);
    double sp = sin(angles.pitch * 0.5);
    double cr = cos(angles.roll * 0.5);
    double sr = sin(angles.roll * 0.5);

    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}

Stim300DriverNode::Stim300DriverNode() : Node("stim300_driver_node"), gravity_(9.80665), sample_rate_(125)
{
    this->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
    this->declare_parameter<double>("variance_gyro", 0.0001 * 2 * 4.6 * pow(10, -4));
    this->declare_parameter<double>("variance_acc", 0.000055);
    this->declare_parameter<double>("sample_rate", 125.0);
    this->declare_parameter<double>("gravity", 9.80665);

    this->get_parameter("device_name", device_name_);
    this->get_parameter("variance_gyro", variance_gyro_);
    this->get_parameter("variance_acc", variance_acc_);
    this->get_parameter("sample_rate", sample_rate_);
    this->get_parameter("gravity", gravity_);

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1000);
    calibration_service_ = this->create_service<std_srvs::srv::Trigger>(
        "IMU_calibration", std::bind(&Stim300DriverNode::responseCalibrateIMU, this, std::placeholders::_1, std::placeholders::_2));

    stim300msg_.angular_velocity_covariance[0] = 0.0000027474;
    stim300msg_.angular_velocity_covariance[4] = 0.0000027474;
    stim300msg_.angular_velocity_covariance[8] = 0.000007312;
    stim300msg_.linear_acceleration_covariance[0] = 0.00041915;
    stim300msg_.linear_acceleration_covariance[4] = 0.00041915;
    stim300msg_.linear_acceleration_covariance[8] = 0.000018995;
    stim300msg_.orientation.x = 0.00000024358;
    stim300msg_.orientation.y = 0.00000024358;
    stim300msg_.orientation.z = 0.00000024358;
    stim300msg_.header.frame_id = "imu_0";

    try
    {
        serial_driver_ = std::make_unique<SerialUnix>(device_name_, stim_const::BaudRate::BAUD_921600);
        driver_stim300_ = std::make_unique<DriverStim300>(*serial_driver_);
        RCLCPP_INFO(this->get_logger(), "STIM300 IMU driver initialized successfully");

        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / (sample_rate_ * 2))),
                                            std::bind(&Stim300DriverNode::timerCallback, this));
    }
    catch (std::runtime_error &error)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", error.what());
    }
}

void Stim300DriverNode::timerCallback()
{
    int difference_in_dataGram{0};
    int count_messages{0};
    int number_of_samples{0};
    double inclination_x{0};
    double inclination_y{0};
    double inclination_z{0};

    double average_calibration_roll{0};
    double average_calibration_pitch{0};
    double inclination_x_calibration_sum{0};
    double inclination_y_calibration_sum{0};
    double inclination_z_calibration_sum{0};
    double inclination_x_average{0};
    double inclination_y_average{0};
    double inclination_z_average{0};

    // Acc wild point filter
    std::vector<double> acceleration_buffer_x{};
    std::vector<double> acceleration_buffer_y{};
    std::vector<double> acceleration_buffer_z{};
    double dropped_acceleration_x_msg{0.0};
    double dropped_acceleration_y_msg{0.0};
    double dropped_acceleration_z_msg{0.0};

    // Gyro wild point filter
    std::vector<double> gyro_buffer_x{};
    std::vector<double> gyro_buffer_y{};
    std::vector<double> gyro_buffer_z{};
    double dropped_gyro_x_msg{0.0};
    double dropped_gyro_y_msg{0.0};
    double dropped_gyro_z_msg{0.0};

    switch (driver_stim300_->update())
    {
        case Stim300Status::NORMAL:
            break;
        case Stim300Status::OUTSIDE_OPERATING_CONDITIONS:
            RCLCPP_DEBUG(this->get_logger(), "Stim 300 outside operating conditions");
            break;
        case Stim300Status::NEW_MEASURMENT:
            inclination_x = driver_stim300_->getIncX();
            inclination_y = driver_stim300_->getIncY();
            inclination_z = driver_stim300_->getIncZ();
            Quaternion q;
            EulerAngles RPY;
            if (calibration_mode_ == true)
            {
                if (number_of_samples < NUMBER_OF_CALIBRATION_SAMPLES)
                {
                    number_of_samples++;
                    inclination_x_calibration_sum += inclination_x;
                    inclination_y_calibration_sum += inclination_y;
                    inclination_z_calibration_sum += inclination_z;
                }
                else
                {
                    inclination_x_average = inclination_x_calibration_sum / NUMBER_OF_CALIBRATION_SAMPLES;
                    inclination_y_average = inclination_y_calibration_sum / NUMBER_OF_CALIBRATION_SAMPLES;
                    inclination_z_average = inclination_z_calibration_sum / NUMBER_OF_CALIBRATION_SAMPLES;

                    average_calibration_roll = atan2(inclination_y_average, inclination_z_average);
                    average_calibration_pitch = atan2(-inclination_x_average, sqrt(pow(inclination_y_average, 2) + pow(inclination_z_average, 2)));
                    std::cout << average_calibration_roll << std::endl;
                    std::cout << average_calibration_pitch << std::endl;
                    RCLCPP_INFO(this->get_logger(), "roll: %f", average_calibration_roll);
                    RCLCPP_INFO(this->get_logger(), "pitch: %f", average_calibration_pitch);
                    RCLCPP_INFO(this->get_logger(), "IMU Calibrated");
                    calibration_mode_ = false;
                }
                break;
            }
            else
            {
                RPY.roll = atan2(inclination_y, inclination_z);
                RPY.pitch = atan2(-inclination_x, sqrt(pow(inclination_y, 2) + pow(inclination_z, 2)));
                q = FromRPYToQuaternion(RPY);

                // Acceleration wild point filter
                acceleration_buffer_x.push_back(driver_stim300_->getAccX() * gravity_);
                acceleration_buffer_y.push_back(driver_stim300_->getAccY() * gravity_);
                acceleration_buffer_z.push_back(driver_stim300_->getAccZ() * gravity_);
                stim300msg_.header.stamp = this->now();

                if (acceleration_buffer_x.size() == 2 && acceleration_buffer_y.size() == 2 && acceleration_buffer_z.size() == 2)
                {
                    if (std::abs(acceleration_buffer_x.back() - acceleration_buffer_x.front()) < ACC_TOLERANCE || dropped_acceleration_x_msg > MAX_DROPPED_ACC_X_MSG)
                    {
                        stim300msg_.linear_acceleration.x = acceleration_buffer_x.back();
                        dropped_acceleration_x_msg = 0;
                    }
                    else
                    {
                        RCLCPP_DEBUG(this->get_logger(), "ACC_X_MSG wild point rejected");
                        dropped_acceleration_x_msg += 1;
                    }

                    if (std::abs(acceleration_buffer_y.back() - acceleration_buffer_y.front()) < ACC_TOLERANCE || dropped_acceleration_y_msg > MAX_DROPPED_ACC_Y_MSG)
                    {
                        stim300msg_.linear_acceleration.y = acceleration_buffer_y.back();
                        dropped_acceleration_y_msg = 0;
                    }
                    else
                    {
                        RCLCPP_DEBUG(this->get_logger(), "ACC_Y_MSG wild point rejected");
                        dropped_acceleration_y_msg += 1;
                    }

                    if (std::abs(acceleration_buffer_z.back() - acceleration_buffer_z.front()) < ACC_TOLERANCE || dropped_acceleration_z_msg > MAX_DROPPED_ACC_Z_MSG)
                    {
                        stim300msg_.linear_acceleration.z = acceleration_buffer_z.back();
                        dropped_acceleration_z_msg = 0;
                    }
                    else
                    {
                        RCLCPP_DEBUG(this->get_logger(), "ACC_Z_MSG wild point rejected");
                        dropped_acceleration_z_msg += 1;
                    }
                    // Empty acceleration buffers
                    acceleration_buffer_x = std::vector<double>{acceleration_buffer_x.back()};
                    acceleration_buffer_y = std::vector<double>{acceleration_buffer_y.back()};
                    acceleration_buffer_z = std::vector<double>{acceleration_buffer_z.back()};
                }
                else
                {
                    stim300msg_.linear_acceleration.x = driver_stim300_->getAccX() * gravity_;
                    stim300msg_.linear_acceleration.y = driver_stim300_->getAccY() * gravity_;
                    stim300msg_.linear_acceleration.z = driver_stim300_->getAccZ() * gravity_;
                }

                // Gyro wild point filter
                gyro_buffer_x.push_back(driver_stim300_->getGyroX());
                gyro_buffer_y.push_back(driver_stim300_->getGyroY());
                gyro_buffer_z.push_back(driver_stim300_->getGyroZ());

                if (gyro_buffer_x.size() == 2 && gyro_buffer_y.size() == 2 && gyro_buffer_z.size() == 2)
                {
                    if (std::abs(gyro_buffer_x.back() - gyro_buffer_x.front()) < std::max(2 * std::abs(gyro_buffer_x.front()), GYRO_X_PEAK_TO_PEAK_NOISE) || dropped_gyro_x_msg > MAX_DROPPED_GYRO_X_MSG)
                    {
                        stim300msg_.angular_velocity.x = gyro_buffer_x.back();
                        dropped_gyro_x_msg = 0;
                    }
                    else
                    {
                        RCLCPP_DEBUG(this->get_logger(), "GYRO_X_MSG wild point rejected");
                        dropped_gyro_x_msg += 1;
                    }

                    if (std::abs(gyro_buffer_y.back() - gyro_buffer_y.front()) < std::max(2 * std::abs(gyro_buffer_y.front()), GYRO_Y_PEAK_TO_PEAK_NOISE) || dropped_gyro_y_msg > MAX_DROPPED_GYRO_Y_MSG)
                    {
                        stim300msg_.angular_velocity.y = gyro_buffer_y.back();
                        dropped_gyro_y_msg = 0;
                    }
                    else
                    {
                        RCLCPP_DEBUG(this->get_logger(), "GYRO_Y_MSG wild point rejected");
                        dropped_gyro_y_msg += 1;
                    }

                    if (std::abs(gyro_buffer_z.back() - gyro_buffer_z.front()) < std::max(2 * std::abs(gyro_buffer_z.front()), GYRO_Z_PEAK_TO_PEAK_NOISE) || dropped_gyro_z_msg > MAX_DROPPED_GYRO_Z_MSG)
                    {
                        stim300msg_.angular_velocity.z = gyro_buffer_z.back();
                        dropped_gyro_z_msg = 0;
                    }
                    else
                    {
                        RCLCPP_DEBUG(this->get_logger(), "GYRO_Z_MSG wild point rejected");
                        dropped_gyro_z_msg += 1;
                    }

                    // Empty buffers
                    gyro_buffer_x = std::vector<double>{gyro_buffer_x.back()};
                    gyro_buffer_y = std::vector<double>{gyro_buffer_y.back()};
                    gyro_buffer_z = std::vector<double>{gyro_buffer_z.back()};
                }
                else
                {
                    stim300msg_.angular_velocity.x = driver_stim300_->getGyroX();
                    stim300msg_.angular_velocity.y = driver_stim300_->getGyroY();
                    stim300msg_.angular_velocity.z = driver_stim300_->getGyroZ();
                }

                stim300msg_.orientation.w = q.w;
                stim300msg_.orientation.x = q.x;
                stim300msg_.orientation.y = q.y;
                stim300msg_.orientation.z = q.z;
                imu_publisher_->publish(stim300msg_);
                break;
            }
        case Stim300Status::CONFIG_CHANGED:
            RCLCPP_INFO(this->get_logger(), "Updated Stim 300 imu config: ");
            RCLCPP_INFO(this->get_logger(), "%s", driver_stim300_->printSensorConfig().c_str());
            sample_rate_ = driver_stim300_->getSampleRate() * 2;
            timer_->cancel();
            timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / (sample_rate_))),
                                                std::bind(&Stim300DriverNode::timerCallback, this));
            break;
        case Stim300Status::STARTING_SENSOR:
            RCLCPP_INFO(this->get_logger(), "Stim 300 IMU is warming up.");
            break;
        case Stim300Status::SYSTEM_INTEGRITY_ERROR:
            RCLCPP_WARN(this->get_logger(), "Stim 300 IMU system integrity error.");
            break;
        case Stim300Status::OVERLOAD:
            RCLCPP_WARN(this->get_logger(), "Stim 300 IMU overload.");
            break;
        case Stim300Status::ERROR_IN_MEASUREMENT_CHANNEL:
            RCLCPP_WARN(this->get_logger(), "Stim 300 IMU error in measurement channel.");
            break;
        case Stim300Status::ERROR:
            RCLCPP_WARN(this->get_logger(), "Stim 300 IMU: internal error.");
    }
}

bool Stim300DriverNode::responseCalibrateIMU(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!calibration_mode_)
    {
        calibration_mode_ = true;
        response->message = "IMU in calibration mode";
        response->success = true;
    }
    
    return true;
}
