#ifndef VEHICLE_SENSOR_DATA_READER_HPP
#define VEHICLE_SENSOR_DATA_READER_HPP

#include <vector>
#include <string>

struct VehicleSensorData {
    std::string sensorName;
    std::string sequenceNumber;
    double creationTimestamp;
    std::string modelName;
    double maxSteeringWheelAngle;
    double maxLeftWheelAngle;
    double maxRightWheelAngle;
    double steeringWheelOffset;
    double steeringWheelSteps;
    double bboxCenterX;
    double bboxCenterY;
    double bboxCenterZ;
    double bboxExtentX;
    double bboxExtentY;
    double bboxExtentZ;
    std::vector<double> bodyTransform;
    std::vector<double> chassisTransform;
    std::vector<double> rtTransform;
    double velocityX;
    double velocityY;
    double velocityZ;
    double accelerationX;
    double accelerationY;
    double accelerationZ;
    double angularVelocityX;
    double angularVelocityY;
    double angularVelocityZ;
    double isActive;
    double isLateralEngaged;
    double isLongitudinalEngaged;
    double useTurnLightModifier;
    double useSteeringWheelTorqueReqModifier;
    double useSteeringWheelAngleReqModifier;
    double useAccelDecelModifier;
    double useEmergencyBrakeModifier;
    double useTransmissionModifier;
    std::string gear;
    double steerAngleRad;
    double vehicleSpeedMps;
    double yawRateRps;
    std::vector<std::string> wheelNames;
    std::vector<double> wheelAnglesRad;
    std::vector<double> wheelLinearSpeeds;
    std::vector<double> wheelIsOnRoad;
    double accelPedalPosition;
    double brakePedalPosition;
    double longitudinalAccelerationMpss;
    double lateralAccelerationMpss;
    std::string beamLightState;
    std::string fogLightState;
    double leftTurnLightsOn;
    double rightTurnLightsOn;
    double hazardLightsOn;
    double hornState;
};

class VehicleSensorDataReader {
public:
    VehicleSensorDataReader(const std::string& vehicleSensorDirPath);

    std::vector<VehicleSensorData > getVehicleSensorData();

private:
    void parseVehicleSensorData(const std::string& filePath);

    std::vector<VehicleSensorData> vehicleSensorDataVec_;
};

#endif // VEHICLE_SENSOR_DATA_READER_HPP
