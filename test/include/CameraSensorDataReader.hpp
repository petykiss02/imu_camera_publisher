#ifndef CAMERA_SENSOR_DATA_READER_HPP
#define CAMERA_SENSOR_DATA_READER_HPP

#include <vector>
#include <string>

struct ImageData {
    std::string fileName;
    int width;
    int height;
    std::vector<uint8_t> pixels;
};

class CameraSensorDataReader {
public:
    CameraSensorDataReader(const std::string& cameraSensorDirPath);

    std::vector<ImageData> getCameraSensorData();

private:
    void parseCameraSensorData(const std::string& filePath);

    std::vector<ImageData> imageSensorDataVec_;
};

#endif // CAMERA_SENSOR_DATA_READER_HPP
