#include "VehicleSensorDataReader.hpp"
#include "json.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <dirent.h>

using json = nlohmann::json;

VehicleSensorDataReader::VehicleSensorDataReader(const std::string& vehicleSensorDirPath) {
    DIR* vehicleDir = opendir(vehicleSensorDirPath.c_str());
    if (!vehicleDir) {
        std::cerr << "Failed to open vehicle sensor data directory: " << vehicleSensorDirPath << std::endl;
        return;
    }

    dirent* dirEntry;
    while ((dirEntry = readdir(vehicleDir)) != nullptr) {
        std::string fileName = dirEntry->d_name;
        if (fileName != "." && fileName != "..") {
            std::string filePath = vehicleSensorDirPath + "/" + fileName;
            parseVehicleSensorData(filePath);
        }
    }
    closedir(vehicleDir);

    std::sort(vehicleSensorDataVec_.begin(), vehicleSensorDataVec_.end(),
              [](const VehicleSensorData& a, const VehicleSensorData& b) {
                  return a.creationTimestamp < b.creationTimestamp;
              });
}

void VehicleSensorDataReader::parseVehicleSensorData(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Failed to open vehicle sensor data file: " << filePath << std::endl;
        return;
    }

    json jsonData;
    file >> jsonData;
    file.close();

    VehicleSensorData vehicleData;
    vehicleData.sensorName = jsonData["sensor_name"];
    vehicleData.sequenceNumber = jsonData["sequence_number"];
    vehicleData.creationTimestamp = jsonData["creation_timestamp"];
    vehicleData.modelName = jsonData["vehicle_info"]["model_name"];
    vehicleData.maxSteeringWheelAngle = jsonData["vehicle_info"]["max_steering_wheel_angle_rad_one_side"];
    vehicleData.maxLeftWheelAngle = jsonData["vehicle_info"]["max_left_wheel_angle_rad_one_side"];
    vehicleData.maxRightWheelAngle = jsonData["vehicle_info"]["max_right_wheel_angle_rad_one_side"];
    vehicleData.steeringWheelOffset = jsonData["vehicle_info"]["steering_wheel_offset"];
    vehicleData.steeringWheelSteps = jsonData["vehicle_info"]["steering_wheel_steps"];
    vehicleData.bboxCenterX = jsonData["vehicle_info"]["bbox_center_x"];
    vehicleData.bboxCenterY = jsonData["vehicle_info"]["bbox_center_y"];
    vehicleData.bboxCenterZ = jsonData["vehicle_info"]["bbox_center_z"];
    vehicleData.bboxExtentX = jsonData["vehicle_info"]["bbox_extent_x"];
    vehicleData.bboxExtentY = jsonData["vehicle_info"]["bbox_extent_y"];
    vehicleData.bboxExtentZ = jsonData["vehicle_info"]["bbox_extent_z"];

    const auto& bodyTransform = jsonData["vehicle_transform"]["body_transform"];
    for (const auto& value : bodyTransform) {
        vehicleData.bodyTransform.push_back(value);
    }

    const auto& chassisTransform = jsonData["vehicle_transform"]["chassis_transform"];
    for (const auto& value : chassisTransform) {
        vehicleData.chassisTransform.push_back(value);
    }

    const auto& rtTransform = jsonData["ego_motion"]["rt_transform"];
    for (const auto& value : rtTransform) {
        vehicleData.rtTransform.push_back(value);
    }

    vehicleData.velocityX = jsonData["ego_motion"]["velocity_x"];
    vehicleData.velocityY = jsonData["ego_motion"]["velocity_y"];
    vehicleData.velocityZ = jsonData["ego_motion"]["velocity_z"];
    vehicleData.accelerationX = jsonData["ego_motion"]["acceleration_x"];
    vehicleData.accelerationY = jsonData["ego_motion"]["acceleration_y"];
    vehicleData.accelerationZ = jsonData["ego_motion"]["acceleration_z"];
    vehicleData.angularVelocityX = jsonData["ego_motion"]["angular_velocity_x"];
    vehicleData.angularVelocityY = jsonData["ego_motion"]["angular_velocity_y"];
    vehicleData.angularVelocityZ = jsonData["ego_motion"]["angular_velocity_z"];

    vehicleData.isActive = jsonData["vehicle_state"]["is_active"];
    vehicleData.isLateralEngaged = jsonData["vehicle_state"]["engage_mode"]["lateral"]["enabled"];
    vehicleData.isLongitudinalEngaged = jsonData["vehicle_state"]["engage_mode"]["longitudinal"]["enabled"];
    vehicleData.useTurnLightModifier = jsonData["vehicle_state"]["engage_mode"]["lateral"]["modifiers"]["use_turnlight"];
    vehicleData.useSteeringWheelTorqueReqModifier = jsonData["vehicle_state"]["engage_mode"]["lateral"]["modifiers"]["use_steering_wheel_torque_req"];
    vehicleData.useSteeringWheelAngleReqModifier = jsonData["vehicle_state"]["engage_mode"]["lateral"]["modifiers"]["use_steering_wheel_angle_req"];
    vehicleData.useAccelDecelModifier = jsonData["vehicle_state"]["engage_mode"]["longitudinal"]["modifiers"]["use_accel_decel"];
    vehicleData.useEmergencyBrakeModifier = jsonData["vehicle_state"]["engage_mode"]["longitudinal"]["modifiers"]["use_emergency_brake"];
    vehicleData.useTransmissionModifier = jsonData["vehicle_state"]["engage_mode"]["longitudinal"]["modifiers"]["use_transmission"];
    vehicleData.gear = jsonData["vehicle_state"]["gear"];
    vehicleData.steerAngleRad = jsonData["vehicle_state"]["steer_angle_rad"];
    vehicleData.vehicleSpeedMps = jsonData["vehicle_state"]["vehicle_speed_mps"];
    vehicleData.yawRateRps = jsonData["vehicle_state"]["yaw_rate_rps"];

    const auto& wheels = jsonData["vehicle_state"]["wheels"];
    for (const auto& wheel : wheels) {
        vehicleData.wheelNames.push_back(wheel["name"]);
        vehicleData.wheelAnglesRad.push_back(wheel["angle_rad"]);
        vehicleData.wheelLinearSpeeds.push_back(wheel["linear_speed"]);
        vehicleData.wheelIsOnRoad.push_back(wheel["is_on_road"]);
    }

    vehicleData.accelPedalPosition = jsonData["vehicle_state"]["accel_pedal_position"];
    vehicleData.brakePedalPosition = jsonData["vehicle_state"]["brake_pedal_position"];
    vehicleData.longitudinalAccelerationMpss = jsonData["vehicle_state"]["longitudinal_acceleration_mpss"];
    vehicleData.lateralAccelerationMpss = jsonData["vehicle_state"]["lateral_acceleration_mpss"];
    vehicleData.beamLightState = jsonData["vehicle_state"]["beam_light_state"];
    vehicleData.fogLightState = jsonData["vehicle_state"]["fog_light_state"];
    vehicleData.leftTurnLightsOn = jsonData["vehicle_state"]["left_turn_lights_on"];
    vehicleData.rightTurnLightsOn = jsonData["vehicle_state"]["right_turn_lights_on"];
    vehicleData.hazardLightsOn = jsonData["vehicle_state"]["hazard_lights_on"];
    vehicleData.hornState = jsonData["vehicle_state"]["horn_state"];

    vehicleSensorDataVec_.push_back(vehicleData);
}

std::vector<VehicleSensorData> VehicleSensorDataReader::getVehicleSensorData() {
    return vehicleSensorDataVec_;
}
