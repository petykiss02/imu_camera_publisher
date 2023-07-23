cmake_minimum_required(VERSION 3.5)
project(imu_camera_publisher)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add your new header and source files to the package
set(HEADER_FILES
    include/ImuSensorDataReader.hpp
    include/CameraSensorDataReader.hpp
    include/VehicleSensorDataReader.hpp
    include/SensorDataPublisher.hpp
)

set(SOURCE_FILES
    src/ImuSensorDataReader.cpp
    src/CameraSensorDataReader.cpp
    src/VehicleSensorDataReader.cpp
    src/SensorDataPublisher.cpp
    src/main.cpp
)

# Add the executable for the main node
add_executable(sensor_data_publisher ${SOURCE_FILES} ${HEADER_FILES})

# Include directories for header files
target_include_directories(sensor_data_publisher PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link the libraries
ament_target_dependencies(sensor_data_publisher
    rclcpp
    std_msgs
    sensor_msgs
)

# Install the executable
install(TARGETS sensor_data_publisher
    DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
    DESTINATION include
)
ament_package()