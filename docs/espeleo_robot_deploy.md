# Espeleo robot deploy — branch `espeleo-jazzy`

This branch contains both **fast_lio** (repo root) and **imu_frame_adapter**
(`imu_frame_adapter/`) for Velodyne VLP-16 + RealSense D456 on ROS 2 Jazzy.

## Clone

```bash
git clone -b espeleo-jazzy git@github.com:MACRO-UFMG/FAST_LIO_ROS2.git
```

## Colcon workspace layout

`imu_frame_adapter` is nested inside this repo. Colcon needs two entries under
`src/`: the FAST-LIO package root and the adapter subfolder.

```bash
mkdir -p ~/fastlio_ws/src
cd ~/fastlio_ws/src
git clone -b espeleo-jazzy git@github.com:MACRO-UFMG/FAST_LIO_ROS2.git
ln -s FAST_LIO_ROS2/imu_frame_adapter imu_frame_adapter
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/fastlio_ws
colcon build --packages-select imu_frame_adapter fast_lio --symlink-install
source install/setup.bash
```

## Run (provisional config)

```bash
# 1. IMU optical → body
ros2 launch imu_frame_adapter imu_adapter.launch.py

# 2. FAST-LIO2 mapping
ros2 launch fast_lio mapping.launch.py \
  config_file:=fast_lio_velodyne_espeleo_provisional.yaml
```

## Robot prerequisites (outside this repo)

- Velodyne driver: `timestamp_first_packet: true`
- RealSense IMU publishing on `/camera/camera/imu`
- Configuration is **provisional** — see header comments in
  `config/fast_lio_velodyne_espeleo_provisional.yaml`
