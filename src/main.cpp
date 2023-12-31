#include "rclcpp/rclcpp.hpp"
#include "SensorDataPublisher.hpp"
#include "ImuSensorDataReader.hpp"
#include "VehicleSensorDataReader.hpp"
#include "CameraSensorDataReader.hpp"

int main(int argc, char * argv[])
{
    std::string imuSensorDirPath = "ego_export/imu_sensor";
    ImuSensorDataReader imuDataReader(imuSensorDirPath);
    std::vector<IMUSensorData> imuSensorDataVec = imuDataReader.getImuSensorData();

    std::string cameraSensorDirPath = "ego_export/pinhole/color";
    CameraSensorDataReader cameraDataReader(cameraSensorDirPath);
    std::vector<ImageData> imageSensorDataVec = cameraDataReader.getCameraSensorData();

    std::string vehicleSensorDirPath = "ego_export/vehicle_sensor";
    VehicleSensorDataReader vehicleDataReader(vehicleSensorDirPath);
    std::vector<VehicleSensorData> vehicleSensorDataVec = vehicleDataReader.getVehicleSensorData();


    rclcpp::init(argc, argv);
    auto sensor_data_publisher = std::make_shared<SensorDataPublisher>(imuSensorDataVec, imageSensorDataVec);
    rclcpp::spin(sensor_data_publisher);
    rclcpp::shutdown();
    return 0;
}
