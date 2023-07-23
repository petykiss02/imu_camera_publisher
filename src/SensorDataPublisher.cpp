#include "SensorDataPublisher.hpp"
#include "rclcpp/logging.hpp"

using namespace std::chrono_literals;

SensorDataPublisher::SensorDataPublisher(const std::vector<IMUSensorData>& imuSensorData,
                                         const std::vector<ImageData>& imageSensorData)
    : Node("sensor_data_publisher"), imuCount_(0), cameraCount_(0),
      imuSensorDataVec_(imuSensorData), imageSensorDataVec_(imageSensorData)
{
    imuPublisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    cameraPublisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_image", 10);

    imuTimer_ = this->create_wall_timer(40ms, std::bind(&SensorDataPublisher::imuTimerCallback, this));
    cameraTimer_ = this->create_wall_timer(40ms, std::bind(&SensorDataPublisher::cameraTimerCallback, this));
}

void SensorDataPublisher::imuTimerCallback()
{
    if (!imuSensorDataVec_.empty())
    {
        size_t index = imuCount_ % imuSensorDataVec_.size(); // Loop back when data is exhausted
        auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

        // Fill in the Imu message with appropriate data
        imu_msg->header.stamp = rclcpp::Time(imuSensorDataVec_[index].creationTimestamp * 1e9); // Convert to nanoseconds
        imu_msg->header.frame_id = "base_link";

        imu_msg->linear_acceleration.x = imuSensorDataVec_[index].accelerationX;
        imu_msg->linear_acceleration.y = imuSensorDataVec_[index].accelerationY;
        imu_msg->linear_acceleration.z = imuSensorDataVec_[index].accelerationZ;

        imu_msg->angular_velocity.z = imuSensorDataVec_[index].angularVelocityZ;

        imu_msg->orientation.x = imuSensorDataVec_[index].orientationNedRoll;
        imu_msg->orientation.y = imuSensorDataVec_[index].orientationNedPitch;
        imu_msg->orientation.z = imuSensorDataVec_[index].orientationNedYaw;


        RCLCPP_INFO(this->get_logger(), "IMU Data:");
        RCLCPP_INFO(this->get_logger(), "  Linear Acceleration: (%f, %f, %f)",
                    imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
        RCLCPP_INFO(this->get_logger(), "  Angular Velocity: (%f, %f, %f)",
                    imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
        RCLCPP_INFO(this->get_logger(), "  Orientation: (%f, %f, %f, %f)",
                    imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
        
        imuPublisher_->publish(std::move(imu_msg));

        imuCount_++;
    }
}

void SensorDataPublisher::cameraTimerCallback()
{
    if (!imageSensorDataVec_.empty())
    {
        size_t index = cameraCount_ % imageSensorDataVec_.size(); // Loop back when data is exhausted
        auto image_msg = std::make_unique<sensor_msgs::msg::Image>();

        image_msg->header.stamp = this->now();
        image_msg->header.frame_id = "base_link";

        image_msg->height = imageSensorDataVec_[index].height;
        image_msg->width = imageSensorDataVec_[index].width;
        image_msg->encoding = "rgba8"; // Assuming RGBA8 format
        image_msg->step = imageSensorDataVec_[index].width * 4; // 4 channels (RGBA8)
        image_msg->data = imageSensorDataVec_[index].pixels;


        RCLCPP_INFO(this->get_logger(), "Camera Image Data:");
        RCLCPP_INFO(this->get_logger(), "  Width: %d, Height: %d", image_msg->width, image_msg->height);
        RCLCPP_INFO(this->get_logger(), "  Encoding: %s", image_msg->encoding.c_str());

        cameraPublisher_->publish(std::move(image_msg));

        cameraCount_++;
    }
}
