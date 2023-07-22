#ifndef IMU_SENSOR_DATA_READER_HPP
#define IMU_SENSOR_DATA_READER_HPP

#include <vector>
#include <string>

struct IMUSensorData {
    std::string sensorName;
    std::string sequenceNumber;
    double creationTimestamp;
    double accelerationX;
    double accelerationY;
    double accelerationZ;
    double angularVelocityZ;
    double orientationNedYaw;
    double orientationNedPitch;
    double orientationNedRoll;
};

class ImuSensorDataReader {
public:
    ImuSensorDataReader(const std::string& imuSensorDirPath);

    std::vector<IMUSensorData> getImuSensorData();

private:
    void parseIMUSensorData(const std::string& filePath);

    std::vector<IMUSensorData> imuSensorDataVec_;
};

#endif // IMU_SENSOR_DATA_READER_HPP
