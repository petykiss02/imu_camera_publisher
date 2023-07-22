#include "ImuSensorDataReader.hpp"
#include "json.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <dirent.h>

using json = nlohmann::json;

ImuSensorDataReader::ImuSensorDataReader(const std::string& imuSensorDirPath) {
    DIR* imuDir = opendir(imuSensorDirPath.c_str());
    if (!imuDir) {
        std::cerr << "Failed to open IMU sensor data directory: " << imuSensorDirPath << std::endl;
        return;
    }

    dirent* dirEntry;
    while ((dirEntry = readdir(imuDir)) != nullptr) {
        std::string fileName = dirEntry->d_name;
        if (fileName != "." && fileName != "..") {
            std::string filePath = imuSensorDirPath + "/" + fileName;
            parseIMUSensorData(filePath);
        }
    }
    closedir(imuDir);

    std::sort(imuSensorDataVec_.begin(), imuSensorDataVec_.end(),
              [](const IMUSensorData& a, const IMUSensorData& b) {
                  return a.creationTimestamp < b.creationTimestamp;
              });
}

void ImuSensorDataReader::parseIMUSensorData(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Failed to open IMU sensor data file: " << filePath << std::endl;
        return;
    }

    json jsonData;
    file >> jsonData;
    file.close();

    IMUSensorData imuData;
    imuData.sensorName = jsonData["sensor_name"];
    imuData.sequenceNumber = jsonData["sequence_number"];
    imuData.creationTimestamp = jsonData["creation_timestamp"];
    imuData.accelerationX = jsonData["acceleration_x"];
    imuData.accelerationY = jsonData["acceleration_y"];
    imuData.accelerationZ = jsonData["acceleration_z"];
    imuData.angularVelocityZ = jsonData["angular_velocity_z"];
    imuData.orientationNedYaw = jsonData["orientation_ned_yaw"];
    imuData.orientationNedPitch = jsonData["orientation_ned_pitch"];
    imuData.orientationNedRoll = jsonData["orientation_ned_roll"];

    imuSensorDataVec_.push_back(imuData);
}

std::vector<IMUSensorData> ImuSensorDataReader::getImuSensorData() {
    return imuSensorDataVec_;
}
