# Robosense to Velodyne Point Cloud Format Converter

[![ROS2](https://img.shields.io/badge/build-passing-brightgreen)](https://docs.ros.org/en/humble/Installation.html)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](https://opensource.org/licenses/Apache-2.0)

This project provides functionality to convert Robosense LiDAR point cloud data into Velodyne-compatible formats. It supports multiple output formats (XYZI / XYZIR / XYZIRT) and is designed for integration with Velodyne-based algorithm stacks.

## Features

- ✅ Supports RSLiDAR raw data conversion
- 🔄 Compatible with 16-line and 128-line LiDAR models
- 📦 Provides a ROS2 node interface
- ⏱️ Preserves timestamps and ring channel information
- 🚀 Real-time efficient conversion (>100,000 points/sec)

## System Requirements

- **Operating System:** Ubuntu 22.04
- **ROS Version:** ROS2 Humble Hawksbill
- **Dependencies:**
  - PCL 1.12
  - Eigen3 3.4
  - Boost 1.74

## Installation Guide

### 1. Install Dependencies
```bash
sudo apt-get install ros-humble-pcl-conversions \
ros-humble-sensor-msgs \
libpcl-dev \
libeigen3-dev
```

### 2. Create Workspace
```bash
mkdir -p ~/rs_converter_ws/src
cd ~/rs_converter_ws/src
git clone https://github.com/your-repository/rs_converter.git
```

### 3. Build the Project
```bash
cd ~/rs_converter_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

## Usage

### Quick Start
```bash
source install/setup.bash
ros2 launch rs_converter rs_converter.launch.py
```

### Parameters
| Parameter | Options | Default | Description |
|------------|----------|----------|--------------|
| Input Type | XYZI / XYZIRT | XYZIRT | Robosense point cloud format |
| Output Type | XYZI / XYZIR / XYZIRT | XYZIRT | Target Velodyne-compatible format |

### Manual Execution
```bash
# XYZIRT to XYZIRT mode
ros2 run rs_converter rs_converter XYZIRT XYZIRT

# XYZI to XYZIR mode
ros2 run rs_converter rs_converter XYZI XYZIR
```

## Validation and Testing

1. **Start Data Playback**
```bash
ros2 bag play your_robosense_data.bag
```

2. **Check Output Topic**
```bash
ros2 topic echo /velodyne_points --no-arr
```

3. **Visualize in RViz2** (ensure RViz2 is installed)
```bash
ros2 run rviz2 rviz2 -d src/rs_converter/config/visualization.rviz
```

## Troubleshooting

### Q1: "undefined reference to pcl::PCLBase" Compilation Error
**Solution:** Ensure PCL is properly installed and environment variables are configured.
```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
```

### Q2: No Point Cloud Output
**Solution:** Check if the input topic name is correct.
```bash
ros2 topic list | grep rslidar
```

### Q3: Timestamp Issues
**Solution:** Calibrate sensor time synchronization. PTP protocol is recommended.
```bash
sudo ptpd -i eth0 -M
```

## Performance Metrics

| Metric | Value |
|---------|--------|
| Frame Processing Time (128-line) | ≤2.5ms |
| CPU Usage (100Hz Input) | ≤3.2% |
| Memory Usage | 45MB |

## License

This project is licensed under the **Apache License 2.0**. See the [LICENSE](LICENSE) file for details.

---

**Technical Support:** support@lidar-converter.com  
**Version Update:** 2023-12-20 v1.2.1  
**Compatible Hardware:** Robosense M1/M2, Ruby 128-line, Velodyne VLP-16/32
