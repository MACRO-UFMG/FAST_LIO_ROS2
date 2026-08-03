# IMU Frame Adapter

Republishes RealSense D456 optical-frame IMU data in a body-aligned frame for FAST-LIO2.

## Frame convention

RealSense optical frame (`camera_imu_optical_frame`):

- X: right, Y: down, Z: forward

Body frame (`camera_imu_body_frame`):

- X: forward, Y: left, Z: up

Fixed rotation (no TF lookup):

```text
x_body =  z_optical
y_body = -x_optical
z_body = -y_optical

R_BO = [[ 0,  0,  1],
        [-1,  0,  0],
        [ 0, -1,  0]]
```

Covariance: `C_body = R_BO @ C_optical @ R_BOᵀ`

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /home/diogo/debug_fastlio2/ws
colcon build --packages-select imu_frame_adapter --symlink-install
source install/setup.bash
```

## Run standalone

```bash
ros2 launch imu_frame_adapter imu_adapter.launch.py use_sim_time:=true
```

## Run with rosbag replay

```bash
# Terminal 1
ros2 launch imu_frame_adapter imu_adapter.launch.py use_sim_time:=true

# Terminal 2 — sensor topics only
ros2 bag play /home/diogo/debug_fastlio2/bag_teste_map_fastlio2 --clock \
  --topics /camera/camera/imu
```

## Tests

```bash
cd /home/diogo/debug_fastlio2/ws
colcon test --packages-select imu_frame_adapter
colcon test-result --verbose
```

Python transform tests (optional):

```bash
pytest /home/diogo/debug_fastlio2/tools/imu_frame_adapter/test/test_transform.py -q
```

## FAST-LIO body-frame extrinsics

When subscribing to `/camera/camera/imu_body`:

```yaml
imu_topic: "/camera/camera/imu_body"
extrinsic_T: [-0.06898, 0.01022, 0.05760]
extrinsic_R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
```
