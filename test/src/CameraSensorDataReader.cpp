#include "CameraSensorDataReader.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <dirent.h>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

CameraSensorDataReader::CameraSensorDataReader(const std::string& cameraSensorDirPath) {
    DIR* cameraDir = opendir(cameraSensorDirPath.c_str());
    if (!cameraDir) {
        std::cerr << "Failed to open camera sensor data directory: " << cameraSensorDirPath << std::endl;
        return;
    }

    dirent* dirEntry;
    while ((dirEntry = readdir(cameraDir)) != nullptr) {
        std::string fileName = dirEntry->d_name;
        if (fileName != "." && fileName != "..") {
            std::string filePath = cameraSensorDirPath + "/" + fileName;
            parseCameraSensorData(filePath);
        }
    }
    closedir(cameraDir);

    std::sort(imageSensorDataVec_.begin(), imageSensorDataVec_.end(),
              [](const ImageData& a, const ImageData& b) {
                  return a.fileName < b.fileName;
              });
}

void CameraSensorDataReader::parseCameraSensorData(const std::string& filePath) {
    int width, height, channels;
    unsigned char* data = stbi_load(filePath.c_str(), &width, &height, &channels, STBI_rgb_alpha);

    if (data != nullptr) {
        ImageData imageData;
        imageData.fileName = filePath;
        imageData.width = width;
        imageData.height = height;
        imageData.pixels.assign(data, data + width * height * 4); // 4 channels (RGBA8)

        imageSensorDataVec_.push_back(imageData);

        stbi_image_free(data);
    } else {
        std::cerr << "Failed to load image file: " << filePath << std::endl;
    }
}

std::vector<ImageData> CameraSensorDataReader::getCameraSensorData() {
    return imageSensorDataVec_;
}
