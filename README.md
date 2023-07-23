
# imu_camera_publisher
This project is a ROS2-based sensor data publisher that reads IMU sensor data from JSON files and camera sensor data from image files and publishes them using ROS2 messages. The data is published to the topics imu `(sensor_msgs/Imu)` and camera_image `(sensor_msgs/Image)`.
## Dependencies

-  ROS2 (Iron Irwini)
-   [stb image loader](https://github.com/nothings/stb/tree/master) library
-   [nlohmann/json](https://github.com/nlohmann/json) library

## Usage

 1. **Source ROS 2 environment:**
```shell
source /opt/ros/iron/setup.bash
```

2.  **Create a new directory:**  
```shell
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

3. **Clone the repository:**  
```shell
git clone https://github.com/petykiss02/imu_camera_publisher.git
```

4.  **From the root of your workspace  build the package using colcon:**  
```shell
cd ~/ros2_ws
colcon build
```

5. **Run the sensor data publisher:**
```shell
. install/setup.bash
ros2 run imu_camera_publisher sensor_data_publisher
```
  
The sensor data publisher will read IMU sensor data from the `ego_export/imu_sensor` directory and camera sensor data from the `ego_export/pinhole/color` directory. Ensure that you have valid sensor data files in these directories before running the publisher. Place the `ego_export` directory in your ROS2 workspace directory. This directory should contain the IMU sensor data in JSON format and camera sensor data in TGA image format.

## Data Format
### IMU Sensor Data
The IMU sensor data is expected to be in JSON format, with the following fields:  
- `sensor_name:` Name of the sensor.
- `sequence_number:` Sequence number of the data.
- `creation_timestamp:` Timestamp of data creation.
- `acceleration_x, acceleration_y, acceleration_z:` Linear acceleration values in X, Y, and Z axes.
- `angular_velocity_z:` Angular velocity value around the Z-axis.
- `orientation_ned_yaw, orientation_ned_pitch, orientation_ned_roll:` Orientation values in NED (North-East-Down) format.

### Camera Sensor Data
The camera sensor data is expected to be in TGA image format (RGBA8). The publisher will load the TGA image files and publish them as ROS2 `sensor_msgs/Image` messages.
	
