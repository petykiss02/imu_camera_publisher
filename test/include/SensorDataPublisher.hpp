#ifndef SENSOR_DATA_PUBLISHER_HPP
#define SENSOR_DATA_PUBLISHER_HPP

#include <chrono>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "ImuSensorDataReader.hpp"
#include "CameraSensorDataReader.hpp"

class SensorDataPublisher : public rclcpp::Node
{
public:
    SensorDataPublisher(const std::vector<IMUSensorData>& imuSensorData,
                        const std::vector<ImageData>& imageSensorData);

private:
    void imuTimerCallback();
    void cameraTimerCallback();

    rclcpp::TimerBase::SharedPtr imuTimer_;
    rclcpp::TimerBase::SharedPtr cameraTimer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cameraPublisher_;
    size_t imuCount_;
    size_t cameraCount_;
    std::vector<IMUSensorData> imuSensorDataVec_;
    std::vector<ImageData> imageSensorDataVec_;
};

#endif // SENSOR_DATA_PUBLISHER_HPP
