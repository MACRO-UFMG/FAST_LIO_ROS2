# FAST-LIO2 with a Velodyne VLP-16 and RealSense D456 IMU

## Configuration, diagnosis, tuning, and reliability report

Date: 2026-07-24  
Repository revision inspected: `827cd0c` on branch `ros2`

This report is specific to this checkout of `MACRO-UFMG/FAST_LIO_ROS2`, not just
generic upstream FAST-LIO2. It audits:

- `config/velodyne.yaml`;
- the untracked Espeleo profile
  `config/fast_lio_velodyne_espeleo.yaml`;
- `launch/mapping.launch.py`;
- all parameter reads and the relevant preprocessing, synchronization, IMU
  propagation, scan matching, map management, and publication code;
- the upstream ROS 2 Velodyne and RealSense drivers; and
- the VLP-16/D456 sensor constraints.

There is no file named `velodyne_mod.yaml` in this workspace or elsewhere under
`/home/diogo`. The likely Espeleo file is
`config/fast_lio_velodyne_espeleo.yaml`. It is currently untracked by Git, so
the deployment must be checked to prove which installed copy is actually being
loaded.

## Bottom line

This sensor pairing is **viable**, but it is not plug-and-play and it has two
real limitations:

1. a VLP-16 has sparse vertical sampling (16 beams over roughly ±15 degrees),
   which makes long tunnels, ramps, planar corridors, dust, and low-feature
   areas much more degenerate than a denser lidar; and
2. a VLP-16 and a USB RealSense IMU do not share a native hardware clock.
   A constant time offset can be calibrated, but host/USB/UDP timestamp jitter
   and clock drift make the system less robust than a hardware-synchronized
   lidar/IMU.

Neither limitation means FAST-LIO2 is a bad design choice. FAST-LIO2 explicitly
supports spinning lidars and external IMUs. The current symptoms should first
be treated as a **timestamp/data-contract/configuration problem**, not as proof
that the pairing cannot work. FAST-LIO2's direct point-to-map formulation is
described in the [FAST-LIO2 paper](https://arxiv.org/abs/2107.06829), and the
upstream project lists spinning lidar support in its
[README](https://github.com/hku-mars/FAST_LIO).

### Highest-priority findings

| Priority | Finding | Consequence | Required action |
|---|---|---|---|
| P0 | The official ROS 2 Velodyne converter publishes per-point `time` in **seconds**, relative to the scan header. | `timestamp_unit: 2` in `config/velodyne.yaml` compresses a 100 ms scan into about 0.1 ms, almost disabling correct deskew. | With the standard driver, use `timestamp_unit: 0`. The Espeleo profile already does. Verify live values instead of assuming. |
| P0 | The Velodyne driver normally stamps a scan with its **last packet** unless `timestamp_first_packet: true`. | Most relative point times can be negative while this FAST-LIO implementation assumes the cloud stamp is scan start and point offsets are nonnegative. | Set `timestamp_first_packet: true`; verify point `time` spans approximately `0 ... 0.1 s` at 10 Hz. |
| P0 | VLP-16 must use `scan_line: 16`; generic `config/velodyne.yaml` says 32. | Wrong ring count and preprocessing behavior. | Use 16. The Espeleo profile already does. |
| P0 | Espeleo enables `mapping.extrinsic_est_en: true` even though an extrinsic is supplied. | Online estimation can move a good calibration, especially while absorbing timing errors or weak excitation. | Set false after offline spatial and temporal calibration. |
| P0 | `common.time_sync_en` does not synchronize standard Velodyne `PointCloud2` in this fork. | Enabling it would provide false confidence; its offset estimator is only reached by the Livox callback. | Keep false. Calibrate `time_offset_lidar_to_imu` and fix clocking at the drivers. |
| P0 | RealSense must publish a **combined** IMU message with both acceleration and angular velocity. | Gyro-only or accel-only topics contain zeros in the other half and cannot drive LIO correctly. | Use `/camera/camera/imu`, `unite_imu_method: 2`, and verify both vectors and rates live. |
| P0 | FAST-LIO's IMU subscription uses a depth-10 default reliable QoS, not sensor-data QoS. | It may be incompatible with a best-effort RealSense publisher or drop samples during bursts/load. | Inspect publisher/subscriber QoS. Match them; preferably patch FAST-LIO to sensor-data QoS with an adequate queue or configure the publisher reliable. |
| P1 | Espeleo uses `point_filter_num: 3`; generic Velodyne uses 4. | This discards 2/3 or 3/4 of an already sparse 16-beam cloud before matching. | Start at 1 for quality, then test 2 only if CPU requires it. |
| P1 | `blind: 2.0` in generic Velodyne is unnecessarily large; Espeleo's `0.5` is below the VLP-16's useful minimum. | The former throws away useful near geometry; the latter may admit invalid near returns. | Start around 0.9–1.0 m and align the driver `min_range`; validate the particular sensor. |
| P1 | `cube_side_length: 1000` and `det_range: 100` retain a very large local map for cave/tunnel use. | More memory and search cost, stale far geometry, less real-time margin. | Limit points in the driver and test roughly `det_range: 30–50`, `cube_side_length: 150–250`. |
| P1 | Espeleo enables path, map, dense global scan, and body scan publication. | CPU, copies, DDS bandwidth, and ever-growing map-publication storage can starve the estimator. | Disable every output not consumed by the stack, especially `map_en` and dense duplicate clouds. |
| P1 | IMU initialization ends after only slightly more than 10 IMU samples in this fork. At normal D456 rates those samples can all be contained in the first lidar measurement group. | A very short, non-stationary sample can corrupt gravity and gyro-bias initialization. | Start the sensors while completely still and remain still until `IMU Initial Done`; preferably add a longer, time- and stationarity-gated initializer. |

The two most important measurements are therefore:

1. prove that each lidar cloud is stamped at scan start and has valid per-point
   offsets from about 0 to 0.1 seconds; and
2. prove that the combined D456 IMU and lidar stamps share a stable timebase
   after applying one calibrated offset.

Do not tune voxel sizes or noise covariance until those two contracts pass.

### Audit of the current Espeleo profile

The current matrix is numerically a valid proper rotation
(`R Rᵀ = I`, `det(R) = +1`) and the translation magnitude is about 9.0 cm.
That only proves mathematical validity; it does not prove the direction,
source frames, timing, or physical lever arm.

| Current Espeleo setting | Assessment |
|---|---|
| `feature_extract_enable: false` | Correct for FAST-LIO2's direct pipeline. |
| `point_filter_num: 3` | Too aggressive as the first baseline for a sparse VLP-16. |
| `max_iteration: 3` | Reasonable starting value. |
| `filter_size_surf: 0.3` | Potentially coarse for close underground structure; sweep downward. |
| `filter_size_map: 0.2` | Plausible; evaluate jointly with the scan voxel. |
| `cube_side_length: 1000` | Oversized for local underground odometry. |
| `scan_line: 16` | Correct. |
| `scan_rate: 10` | Correct only if the physical lidar is at 600 RPM. |
| `timestamp_unit: 0` | Correct for the standard ROS 2 Velodyne converter. |
| `blind: 0.5` | Below the VLP-16 useful near range; align around 0.9–1.0 m unless measurements support another value. |
| four covariance values at upstream defaults | Uncalibrated placeholders, not D456-specific values. |
| `det_range: 100` | Larger than normally useful underground and not an actual input range cutoff. |
| `extrinsic_est_en: true` | Change to false after offline calibration. |
| `time_offset_lidar_to_imu: 0` | Valid only if measurement proves a negligible offset; otherwise calibrate it. |
| path, map, dense scan, and body scan all enabled | Excessive for a reliability baseline. Disable unused outputs. |
| `pcd_save_en: false` | Correct for production. |
| `load_map_file_path`, `use_predifined_map` | Not consumed by the inspected mapping node. |

## Recommended starting profile

This is an experimental baseline, not a calibration result. Replace the
extrinsic and time offset with measured values. It intentionally minimizes
outputs and preserves VLP-16 points while the system is being characterized.

```yaml
/**:
  ros__parameters:
    feature_extract_enable: false
    point_filter_num: 1
    max_iteration: 3
    filter_size_surf: 0.15
    filter_size_map: 0.25
    cube_side_length: 200.0
    runtime_pos_log_enable: false
    map_file_path: ""

    common:
      lid_topic: "/velodyne_points"
      imu_topic: "/camera/camera/imu"
      time_sync_en: false
      # The code applies: adjusted_imu_stamp = raw_imu_stamp - this_value.
      time_offset_lidar_to_imu: 0.0  # replace with calibrated value

    preprocess:
      lidar_type: 2
      scan_line: 16
      scan_rate: 10
      # Correct only after verifying the standard ROS 2 Velodyne `time` field
      # is floating-point seconds.
      timestamp_unit: 0
      blind: 0.9

    mapping:
      # Temporary starting values only; replace from stationary IMU analysis.
      acc_cov: 0.1
      gyr_cov: 0.1
      b_acc_cov: 0.0001
      b_gyr_cov: 0.0001
      fov_degree: 360.0       # no operational effect in this checkout
      det_range: 40.0
      extrinsic_est_en: false
      # LiDAR pose in the exact IMU message frame:
      # p_imu = extrinsic_R * p_lidar + extrinsic_T
      extrinsic_T: [-0.01022, -0.05760, -0.06898]
      extrinsic_R: [0.0, -1.0,  0.0,
                    0.0,  0.0, -1.0,
                    1.0,  0.0,  0.0]

    publish:
      path_en: false
      effect_map_en: false
      map_en: false
      scan_publish_en: true
      dense_publish_en: false
      scan_bodyframe_pub_en: false

    pcd_save:
      pcd_save_en: false
      interval: -1
```

Suggested Velodyne-side starting conditions:

```yaml
velodyne_driver_node:
  ros__parameters:
    model: VLP16
    rpm: 600.0
    timestamp_first_packet: true
    cut_angle: 0.0
    gps_time: false       # true only with a correctly configured GPS/PPS source
    frame_id: velodyne

velodyne_transform_node:
  ros__parameters:
    min_range: 0.9
    max_range: 50.0
    organize_cloud: false
    # Do not transform into a moving target/fixed frame here. FAST-LIO deskews.
```

The `rpm` must match the physical sensor setting. At another RPM, change
`preprocess.scan_rate` accordingly. A fixed `cut_angle` gives complete
revolutions more deterministically than a packet-count boundary, particularly
when return mode changes. Confirm the actual cloud duration rather than
trusting configuration.

## 1. Required sensor-data contract

FAST-LIO is highly sensitive to the *meaning* of incoming fields, not merely
their names.

### 1.1 Velodyne point cloud

For every `/velodyne_points` message:

- `header.stamp` must represent scan start;
- `header.frame_id` must be the frame used by the calibrated extrinsic;
- fields must include `x`, `y`, `z`, `intensity`, `ring`, and `time`;
- `ring` must be in `0 ... 15`;
- `time` must be nonnegative, in seconds with `timestamp_unit: 0`, and cover
  approximately one revolution (`~0.1 s` at 10 Hz);
- point order should be temporal, or at minimum the final point's `time` must
  equal the cloud's maximum time in this checkout;
- scan header deltas should be near 0.1 s without backwards jumps;
- clouds should represent one complete revolution and the configured return
  mode should not unexpectedly halve/double the packet count;
- invalid/NaN points and returns outside the selected range should already be
  removed or handled consistently.

The official ROS 2 Velodyne conversion code declares the point `time` as
floating-point seconds relative to the scan header; see the upstream
[`rawdata.cpp`](https://raw.githubusercontent.com/ros-drivers/velodyne/ros2/velodyne_pointcloud/src/lib/rawdata.cpp).
The driver chooses the first or last packet timestamp in
[`driver.cpp`](https://raw.githubusercontent.com/ros-drivers/velodyne/ros2/velodyne_driver/src/driver/driver.cpp).

There is a local code hazard: scan end time is derived from
`points.back().curvature`, not the maximum per-point time. The preprocessing
stores the converted point time in `curvature` in milliseconds. If the
converter does not emit temporally ordered points, the scan duration is wrong.
For reliability, change this to the maximum time after validating the input.

### 1.2 D456 IMU

For `/camera/camera/imu`:

- both `angular_velocity` and `linear_acceleration` must be populated in every
  message;
- units must be rad/s and m/s²;
- `header.frame_id` must be the exact IMU frame used in the extrinsic;
- axes must be confirmed. RealSense sensor topics use optical coordinates
  (x right, y down, z forward), which are not the conventional ROS
  `camera_link` axes;
- stamps must be monotonic and have a stable rate;
- the rate should normally be the highest stable supported gyro rate, with
  accel interpolated to gyro times;
- messages must not arrive in large bursts or be dropped by QoS/queue pressure;
- the sensor must be calibrated and temperature behavior characterized.

The official RealSense ROS wrapper documents combined IMU publication and
optical-frame conventions in
[`realsense-ros`](https://github.com/realsenseai/realsense-ros). The D456 is a
D455-derived design in an IP65 enclosure according to the
[official product page](https://www.realsenseai.com/products/d456-usb/).
Supported IMU rates/chip revisions should be queried from the actual device;
the [D400 datasheet](https://dev.realsenseai.com/download/42003/) covers
different BMI055/BMI085 revisions.

FAST-LIO ignores the incoming `orientation` and all three
`sensor_msgs/Imu` covariance arrays. Only the six acceleration/gyro values and
timestamp are consumed. RealSense wrapper parameters that merely fill message
covariance do not tune this filter.

### 1.3 Cross-sensor time

The official Velodyne driver can timestamp UDP packets from averaged host
receive time, or lidar/GPS time when properly configured; see
[`input.cpp`](https://raw.githubusercontent.com/ros-drivers/velodyne/ros2/velodyne_driver/src/lib/input.cpp).
RealSense global timestamps map the camera hardware clock to the host time.
Those mechanisms do not hardware-synchronize the two sensors.

Measure:

- constant lidar-to-IMU offset;
- offset repeatability after each launch/reset;
- short-term stamp jitter;
- offset drift over 10–30 minutes and over temperature/load;
- packet/sample loss under full stack CPU and network load.

In this code:

```text
adjusted_imu_stamp = raw_imu_stamp - common.time_offset_lidar_to_imu
```

So the sign must be converted explicitly from the convention used by the
calibration tool. Test a small positive and negative perturbation against a
dynamic bag if the convention is uncertain.

`common.time_sync_en` only estimates a time difference in the Livox custom
message callback in this fork. It has no such effect in the standard Velodyne
`PointCloud2` callback. Leave it false.

## 2. Spatial extrinsic calibration

This implementation expects the LiDAR pose expressed in the IMU message frame:

```text
p_imu = R_LI * p_lidar + T_LI
```

`extrinsic_T` is in metres and `extrinsic_R` is read row-major. Confirm:

- `R * Rᵀ ≈ I`;
- `det(R) ≈ +1`;
- the lever arm sign agrees with a physical measurement;
- the lidar frame is the raw `/velodyne_points` frame;
- the IMU frame is exactly `/camera/camera/imu.header.frame_id`, not
  `camera_link`, an RGB optical frame, or `base_link`;
- no driver-side transform silently changes the cloud before FAST-LIO;
- the transform direction has not been inverted;
- the calibration dataset contains rotations about all three axes and
  translations with strong 3-D structure.

Use an offline joint spatial/temporal calibration such as
[LI-Init](https://github.com/hku-mars/LiDAR_IMU_Init); its method is documented
in the [LI-Init paper](https://arxiv.org/abs/2202.11006). Repeat calibration on
multiple bags and compare the estimates. A result that changes materially
between bags usually indicates inadequate excitation, timing error, flexible
mounting, bad IMU calibration, or degeneracy.

Set `mapping.extrinsic_est_en: false` for normal operation once calibration is
accepted. Online extrinsic estimation is not a substitute for initialization.
When timing is wrong or the motion is weak, it can use extrinsic states to
absorb other errors and then destabilize odometry.

Check the mechanical installation as seriously as the matrix:

- no compliant camera bracket, cable pull, or lidar vibration;
- both sensors rigidly tied to the same chassis member;
- no fan/motor resonance near the IMU;
- no shifting mounting surfaces after impacts;
- repeat a static axis/gravity check after reassembly.

## 3. Every FAST-LIO YAML parameter in this checkout

### Top-level estimator and map parameters

| Parameter | What the code does | Quality/performance guidance |
|---|---|---|
| `feature_extract_enable` | Selects legacy edge/plane feature preprocessing instead of FAST-LIO2 direct points. | Keep `false`. The direct pipeline is intended for FAST-LIO2. In this checkout the feature path also has a constructor defect: `disB` is left uninitialized, so do not enable it without fixing and testing that code. |
| `point_filter_num` | Keeps approximately every Nth valid input point before voxel filtering. | Critical for VLP-16 density. Start 1; test 2 only for CPU. Values 3–4 can cause loss of constraints. |
| `max_iteration` | Maximum iterated-EKF measurement updates per scan. | 3 is a reasonable start. Test 4 if residuals improve and real-time deadline remains safe. Extra iterations cannot fix bad timing or calibration. |
| `filter_size_surf` | Voxel leaf size applied to the current scan before matching. | Smaller preserves detail but raises neighbor-search/update cost. Start 0.10–0.20 m indoors/caves; sweep 0.10, 0.15, 0.20, 0.30. |
| `filter_size_map` | iKD-tree map downsample resolution and voxel logic for inserted points. | Start 0.20–0.30 m for VLP-16 cave work; sweep with `filter_size_surf`. Too small increases memory/cost and noisy duplicate geometry; too large removes structure. The LI-Init example uses 0.05/0.15 m surface/map voxels indoors, but that is guidance, not a universal optimum. |
| `filter_size_corner` | Declared and read but not used by the active direct pipeline. | No odometry effect with `feature_extract_enable: false`. |
| `cube_side_length` | Side length of the moving local-map cube. Old slabs are deleted when the sensor approaches a cube edge. | 1000 m is excessive for local cave odometry. It must be comfortably larger than the detection-range margins; test 150–250 m with `det_range` 30–50 m. |
| `runtime_pos_log_enable` | Enables timing/state logs and large fixed logging arrays. | Keep false in production unless actively profiling; file I/O perturbs real-time behavior. |
| `map_file_path` | Path used by map save/load extensions in this fork. | Does not improve scan matching by itself. Empty when unused. Validate custom map-localization code separately. |
| `load_map_file_path` | Present in the Espeleo YAML but not declared/read in the inspected `laserMapping.cpp`. | No effect in this checkout unless another uninspected build/patch consumes it. |
| `use_predifined_map` | Present in the Espeleo YAML but not declared/read in the inspected `laserMapping.cpp`; also misspelled. | No effect in this checkout. Do not assume localization against a prior map is active. |

### `common`

| Parameter | What the code does | Guidance |
|---|---|---|
| `common.lid_topic` | Subscribes to `sensor_msgs/PointCloud2` for non-Livox lidar. | Must be the native, time-enabled Velodyne cloud. Avoid an intermediate node that strips `time`/`ring`. |
| `common.imu_topic` | Subscribes to `sensor_msgs/Imu`. | Use the combined D456 topic; verify both vectors and exact frame. |
| `common.time_sync_en` | Enables a crude automatic difference only in the Livox callback. | Keep false for Velodyne. It does not solve this pairing's clock problem. |
| `common.time_offset_lidar_to_imu` | Subtracts a constant from every raw IMU timestamp. | Calibrate jointly with extrinsics, define the sign from code, repeat across launches/temperatures, and monitor drift. |

### `preprocess`

| Parameter | What the code does | Guidance for VLP-16 |
|---|---|---|
| `preprocess.lidar_type` | Chooses message parsing: 1 Livox, 2 Velodyne, 3 Ouster. | Must be 2. |
| `preprocess.scan_line` | Number of rings used for bounds/feature structures. | Must be 16. |
| `preprocess.scan_rate` | Expected scan rate; also used as fallback duration when point times appear invalid. | Must match actual rotation rate: usually 10 Hz at 600 RPM. Measure header and point-time durations. |
| `preprocess.timestamp_unit` | Converts `time` to milliseconds stored in PCL `curvature`: 0 seconds, 1 milliseconds, 2 microseconds, 3 nanoseconds. | Standard ROS 2 Velodyne uses 0. A custom driver might differ; decide from the live schema and values. |
| `preprocess.blind` | Rejects points with range below this radius. | Start 0.9–1.0 m; align converter `min_range`. Larger values remove valuable close cave structure. |

### `mapping`

| Parameter | What the code does | Guidance |
|---|---|---|
| `mapping.acc_cov` | Scalar isotropic accelerometer process-noise diagonal used after initialization. | Estimate from calibrated stationary data at the deployed rate. It cannot correct bias, scale, axes, timing, or vibration. |
| `mapping.gyr_cov` | Scalar isotropic gyro process-noise diagonal. | Estimate from stationary data and validate dynamically. Too small makes lidar corrections hard to accept; too large makes prediction noisy. |
| `mapping.b_acc_cov` | Scalar accelerometer-bias random-walk process noise. | Estimate over long static records/Allan analysis. Tune slowly after white noise. |
| `mapping.b_gyr_cov` | Scalar gyro-bias random-walk process noise. | Same principle; temperature warm-up matters for a USB camera IMU. |
| `mapping.fov_degree` | Converted to a cosine threshold but never used by the active map logic. | No operational effect in this checkout. |
| `mapping.det_range` | Controls when/how the local map cube is shifted. | It is **not** an input maximum-range filter. Limit actual lidar points using the Velodyne converter `max_range`. Test 30–50 m indoors/caves. |
| `mapping.extrinsic_est_en` | Includes LiDAR-to-IMU rotation/translation in the update Jacobian. | False after offline calibration. True only in a deliberately observable calibration experiment. |
| `mapping.extrinsic_T` | LiDAR origin expressed in the IMU frame, metres. | Calibrate, confirm direction/frame, and mechanically verify. |
| `mapping.extrinsic_R` | LiDAR axes rotated into IMU axes, row-major 3×3. | Calibrate and verify orthonormality/determinant. |

#### Covariance semantics in this implementation

These four YAML values are not read from `sensor_msgs/Imu.covariance`. The
filter propagates covariance approximately as:

```text
P_next = F P Fᵀ + (dt Fw) Q (dt Fw)ᵀ
```

Thus the implementation's `Q` behaves like a per-sample variance in that
discretization, not a continuous noise-density value copied verbatim from a
datasheet. If a calibrated white-noise density is `n` per square-root Hz, an
initial conversion at sample rate `f` is approximately `Q = n² f`; validate
against innovation consistency. A simpler starting estimate is the per-axis
sample variance from a long stationary record after calibration and warm-up.
Because the YAML supports only one scalar per sensor, use a conservative
representative value when axes differ.

Do not “tune by making the trajectory look smooth.” Evaluate innovations,
stationary drift, repeatability, and ground truth. Very small covariance can
make an attractive but overconfident and wrong result.

### `publish`

| Parameter | Effect | Reliability guidance |
|---|---|---|
| `publish.path_en` | Publishes the accumulated path. | Visualization only; disable in production if unused because the message grows over time. |
| `publish.effect_map_en` | Publishes selected/effective measurement points. | Debug only; disable normally. |
| `publish.map_en` | Publishes accumulated map points from a growing buffer at about 1 Hz in this fork. | Disable in production. This path can grow memory and DDS traffic; it is not required for estimation. |
| `publish.scan_publish_en` | Master gate for registered scan publication. | Disable if no consumer. When false it closes scan outputs. |
| `publish.dense_publish_en` | Publishes full-resolution registered scan rather than the downsampled cloud. | Disable for real-time margin unless density is required. |
| `publish.scan_bodyframe_pub_en` | Publishes an additional undistorted scan in the IMU body frame. | Disable unless consumed; it adds transformation/copy/DDS cost. |

### `pcd_save`

| Parameter | Effect | Guidance |
|---|---|---|
| `pcd_save.pcd_save_en` | Accumulates registered dense points for PCD output. | False in production/long runs. |
| `pcd_save.interval` | Writes one PCD every N frames if positive; `-1` accumulates the entire run into one cloud. | Never use `-1` for a long mission with saving enabled; it can exhaust memory and hurt real-time behavior. |

### ROS/launch controls outside the YAML

| Control | Effect | Guidance |
|---|---|---|
| `config_path`, `config_file` | Select the file actually loaded. | `mapping.launch.py` defaults to `mid360_mod.yaml`, not any Velodyne file. Always pass the Espeleo filename explicitly and verify startup parameters. |
| `use_sim_time` | Uses `/clock` for ROS time. | False for live sensors. True for bag playback only when a valid clock is published and all relevant nodes agree. |
| namespace/node wildcard `/**` | Applies the YAML to all matching nodes. | Convenient, but inspect the node's effective parameters to ensure the intended file won. |
| process scheduling/CPU affinity | Determines callback latency and sample loss. | Profile with the full stack. Reserve CPU, avoid thermal throttling, and keep visualization/recording off the estimator core when possible. |
| ROS QoS and queue depth | Controls compatibility, loss, and latency. | Inspect both topics with `ros2 topic info --verbose`; depth 10 is fragile for high-rate IMU bursts. |
| build type/compiler/OpenMP | Changes throughput and floating-point behavior. | Use the intended Release build. This CMake limits OpenMP threads at compile time (typically 3 on larger x86 systems), so measure rather than assume all cores are used. |

The launch file declares RViz substitutions internally but does not add their
launch arguments or RViz node to the final `LaunchDescription`; changing
`rviz` arguments has no effect in this checkout.

## 4. Hidden code constants and behaviors that affect quality

These are not YAML-tunable without changing and rebuilding the code.

| Code control | Current value/behavior | Why it matters |
|---|---|---|
| IMU initialization count | `MAX_INI_COUNT = 10` IMU samples, not lidar frames | `N` increments for every IMU message, so a 200 Hz stream can exceed the threshold in the first ~0.1 s lidar group. It estimates mean acceleration, mean gyro, gravity direction, and initial gyro bias while assuming stationarity. This is much too short to prove stationarity or characterize bias. |
| Gravity magnitude | 9.81 m/s² | Each acceleration sample is rescaled using the initialization mean norm. Bad motion during initialization therefore contaminates all later acceleration scaling. |
| EKF warm-up | `INIT_TIME = 0.1 s` | Controls early map/update state, separate from IMU initialization. |
| Lidar measurement covariance | `LASER_POINT_COV = 0.001` | Fixed for every matched point. There is no YAML parameter for range-dependent VLP-16 noise or dust/outlier conditions. |
| Nearest neighbors | `NUM_MATCH_POINTS = 5` | Exactly five map neighbors are requested for a plane constraint. |
| Neighbor-distance gate | fifth squared distance ≤ 5 m² | Sparse maps/large voxels may reject candidates; clutter may accept unrelated neighbors. |
| Plane-fit threshold | about 0.1 m | Fixed planar consistency threshold. |
| Residual acceptance | score `1 - 0.9*abs(residual)/sqrt(point_range)` > 0.9 | Fixed range-scaled gate; no robust-kernel YAML control. |
| Iteration convergence epsilon | all 23 state elements use 0.001 | Fixed stopping scale across heterogeneous state units. |
| Selected-point arrays | fixed capacity around 100,000 | Extremely dense scans/small voxels can overflow assumptions. VLP-16 normally stays below this, but guard it in production code. |
| Timer | mapping callback at 100 Hz | Backlog and callback scheduling determine effective latency. |
| Local-map move threshold | `MOV_THRESHOLD = 1.5` | Couples `det_range` and `cube_side_length`; nonsensical combinations can shift/delete poorly. |
| Odometry covariance order | odometry is published before current covariance is filled | Published covariance lags by one update; the first can be zero/stale. Patch before downstream fusion relies on it. |
| Odometry twist | not populated with a meaningful velocity | Do not expect a complete nav odometry message without adapting the publisher. |
| Output frames | hardcoded `camera_init` and `body` | FAST-LIO outputs the IMU-body pose, not automatically `base_link`. Apply the calibrated fixed transform to the robot base; do not merely rename frames. |
| Degeneracy handling | no explicit health/degeneracy status output | A numerically running node can still be unobservable or wrong. Add monitoring/gating before treating it as a reliable stack source. |
| Input buffers | deques grow as callbacks arrive | Sustained processing lag can grow latency/memory; no production watchdog enforces deadline/drop policy. |
| Feature thresholds | numerous fixed group/range/ratio thresholds | Only relevant if legacy feature extraction is enabled; keep it disabled unless the path is repaired and separately tuned. |

### Additional code issues worth fixing

1. Compute scan duration from the maximum valid per-point time, not
   `points.back()`.
2. Reject or flag clouds with negative times, non-finite times, duration outside
   an expected band, wrong ring range, or backwards stamps.
3. Use explicit compatible sensor-data QoS and a measured IMU queue depth.
4. Fill odometry pose covariance before publishing and publish meaningful twist
   or explicitly leave it unavailable to downstream consumers.
5. Publish estimator health: last lidar/IMU age, rate, dropped/backwards
   samples, scan duration, match count/ratio, residual distribution, update
   time, map size, innovation/NIS, and initialization state.
6. Add degeneracy/conditioning detection and a policy for downstream fusion
   (inflate covariance, reject output, or fail over).
7. Lengthen or gate IMU initialization on measured stationarity.
8. Fix the uninitialized `disB` constructor member before anyone enables
   feature extraction.

## 5. Velodyne VLP-16 controls outside FAST-LIO

### Sensor/driver parameters

| Parameter/control | Effect on odometry | Recommendation |
|---|---|---|
| `device_ip` | Selects/filter packets from the physical lidar. | Set explicitly in multi-network systems; detect packet loss/duplicates. |
| `port` | UDP data port. | Match sensor network configuration. |
| `model` | Selects packet timing and calibration model. | `VLP16`. Wrong model corrupts angles and timing. |
| `rpm` | Determines expected packet count/revolution in the driver; does not necessarily command the sensor. | Match sensor web UI. 600 RPM = 10 Hz. |
| `cut_angle` | Ends scans at a configured azimuth instead of only packet count. | Use a fixed azimuth and verify full revolutions; especially useful when return modes alter packet rate. |
| `timestamp_first_packet` | Chooses scan header from first vs last packet. | **True is required by this FAST-LIO time convention.** |
| `gps_time` | Uses sensor/GPS timestamp rather than host receive timing. | False unless GPS/PPS and lidar time are correctly configured and validated. |
| `time_offset` | Driver-level stamp adjustment. | Do not rely on it without version-specific verification; calibrate at one layer only to avoid double correction. |
| `frame_id` | Names raw cloud frame. | Must match spatial calibration source frame. |
| `pcap`, `read_once`, `read_fast`, `repeat_delay` | Offline playback controls. | Ensure playback timing and `/clock` policy reproduce the live contract; `read_fast` can overwhelm queues. |
| `enabled` | Driver packet processing. | Monitor transitions/reconnect behavior. |
| calibration file | Vertical/horizontal angle and distance correction. | Use the correct VLP-16/factory calibration for the unit. Wrong corrections create warped planes. |
| `min_range` | Removes near invalid returns. | Start ~0.9 m and align FAST-LIO `blind`. |
| `max_range` | Removes far/noisy points before LIO. | Set for the environment, often 40–70 m underground rather than the full nominal range. |
| `view_direction`, `view_width` | Crops azimuth sector. | Full 360° normally gives best constraints. Crop only known self-returns/obstructions and validate degeneracy. |
| `organize_cloud` | Produces organized vs compact cloud. | `false` is a safer initial choice for temporal ordering and fewer placeholders/NaNs; still verify last time equals max time. |
| `target_frame`, `fixed_frame` | Applies TF-based transformation/motion compensation in converter variants. | Leave native lidar frame and let FAST-LIO deskew. Double deskew or a moving pre-transform invalidates the extrinsic/time model. |
| return mode | Strongest/last/dual changes point and packet pattern. | Start with one return mode. If dual return is needed for vegetation/dust, verify scan completeness, duplicates, duration, packet-count logic, and CPU. |
| sensor FoV/phase-lock settings | Changes azimuth coverage and phase. | Keep full FoV for LIO; phase lock helps only with a real synchronization design. |
| PPS/GPS qualifiers | Determines validity of lidar clock. | Monitor lock state; never enable GPS time merely because the option exists. |
| network MTU/socket buffers/NIC load | Packet loss creates missing sectors and timestamp irregularity. | Measure packet loss at mission load; isolate NIC/VLAN where possible and increase buffers only with evidence. |

The [VLP-16 user manual](https://data.ouster.io/downloads/velodyne/user-manual/vlp-16-user-manual-revf.pdf)
documents the firing sequence, rotation-rate range, returns, range, and the
16-channel vertical pattern. Driver behavior is version-specific, so record
the exact `ros-drivers/velodyne` commit/package version with each test.

### What one good lidar message should look like at 10 Hz

```text
ring_min = 0
ring_max = 15
time_min ≈ 0.0 s
time_max ≈ 0.10 s
time_last ≈ time_max
negative_time_count = 0
header_delta median ≈ 0.10 s
duration jitter = small and bounded
```

If `time_max ≈ 100000`, the source is probably microseconds and
`timestamp_unit: 2` would be appropriate. If `time_max ≈ 100`, it is probably
milliseconds and use 1. If `time_max ≈ 0.1`, use 0. Do not infer the unit from
the YAML filename.

## 6. RealSense D456 IMU controls outside FAST-LIO

| Parameter/control | Effect | Recommendation |
|---|---|---|
| `enable_gyro`, `enable_accel` | Enables raw inertial streams. | Both true. |
| `gyro_fps`, `accel_fps` | Physical sample rates supported by the installed IMU revision. | Query the device. Use the highest stable gyro rate and a supported accel rate, then measure actual topic rates. |
| `unite_imu_method` | Creates combined IMU: 0 off, 1 copy latest accel, 2 interpolate accel to gyro timestamp. | Use 2 for LIO unless testing reveals a driver-specific problem. |
| `global_time_enabled`/global timestamps | Maps hardware timestamps toward host time. | Enable only if supported/stable for the actual wrapper version; characterize reset/launch offset and drift against Velodyne. |
| IMU QoS | Controls compatibility and loss. | Explicitly match FAST-LIO. Inspect endpoints live. |
| `hold_back_imu_for_frames` | Buffers IMU while image frames/TF are prepared. | Prefer false for low-latency LIO; bursts can overflow FAST-LIO's small queue. |
| `enable_sync` | Synchronizes RealSense image streams. | Does **not** synchronize the D456 IMU to VLP-16. Do not use it as evidence of cross-sensor sync. |
| gyro/accel covariance wrapper settings | Fills ROS message metadata. | No effect on this FAST-LIO because incoming covariance is ignored. |
| `initial_reset`/reconnect behavior | Resets hardware and can change clock epoch/startup transients. | Test repeatability across cold/warm starts. Delay LIO initialization until data rates and temperature settle. |
| serial number/USB port | Selects the intended camera and topology. | Pin the device; avoid USB hubs/contention that create bursts or resets. |
| image stream enables | Consume USB bandwidth and CPU. | If the camera is only an IMU for this process, disable unused streams. If images are needed elsewhere, test under full bandwidth. |
| firmware/librealsense/wrapper versions | Change timestamp, interpolation, QoS, and calibration behavior. | Pin and record known-good versions. Requalify upgrades. |
| IMU calibration | Corrects axis bias/scale/cross-axis errors. | Run the official RealSense IMU calibration flow for the actual unit, write/verify calibration as appropriate, then collect stationary validation data. |
| temperature/warm-up | Changes MEMS bias. | Characterize cold start and warmed operation; initialize after a defined warm-up or increase bias uncertainty. |

The official librealsense
[IMU documentation](https://raw.githubusercontent.com/IntelRealSense/librealsense/master/doc/d435i.md)
describes IMU streams/calibration concepts for D400 inertial devices. Do not
copy a D435i's numerical noise or exact profiles blindly onto a D456; query and
measure the installed hardware.

### Static IMU acceptance checks

At several known orientations after warm-up:

- acceleration norm should be close to local gravity;
- each axis/sign should agree with the RealSense optical-frame convention;
- stationary mean gyro should be near zero after calibration;
- sample intervals should show the requested rate with no large gaps/bursts;
- noise mean/variance should be repeatable across runs;
- vibration with motors on should be separately measured;
- timestamp behavior should survive USB load, camera streams, and reconnect.

Record at least 30–60 minutes for bias/noise/Allan analysis if reliability is
the goal. Use per-axis results even though FAST-LIO ultimately accepts scalar
noise values.

## 7. Environment and observability limitations

FAST-LIO2 is local odometry, not a globally consistent localization system. It
has no loop closure here, so drift over a long cave mission is expected. Use
loop closure/map localization or another global correction layer above it.

The VLP-16 is most vulnerable when the current field of view contains:

- a long straight tunnel with parallel walls;
- one dominant floor/wall plane;
- a ramp with little transverse structure;
- dust/fog/water returns;
- moving people/robots;
- close self-returns or chassis occlusion;
- high angular motion combined with bad point timing;
- motion along a weakly observable axis.

No parameter can create missing geometry. Mounting angle can materially help:
a small deliberate tilt may expose floor, ceiling, and walls more evenly, but
it must be included in the calibrated extrinsic and evaluated for blind zones.

This implementation does not emit a dependable degeneracy/health state. Its
covariance should not be accepted blindly by the rest of the stack. A
production wrapper should gate or inflate odometry based on:

- timestamp/data-contract failures;
- current IMU/lidar age and rate;
- processing deadline misses and buffer growth;
- number/fraction of effective matched points;
- residual/innovation statistics and Hessian/observability conditioning;
- sudden pose/velocity/bias/extrinsic jumps;
- map size and CPU/memory;
- initialization and stationary-status validity.

## 8. Ordered commissioning procedure

Change one class of variables at a time and retain all raw bags.

### Stage A — prove the deployed configuration

1. Launch FAST-LIO with an explicit:

   ```bash
   ros2 launch fast_lio mapping.launch.py \
     config_file:=fast_lio_velodyne_espeleo.yaml use_sim_time:=false
   ```

2. Inspect effective parameters, especially file-specific and nested values:

   ```bash
   ros2 param dump /fastlio_mapping
   ```

3. Record exact Git commits/package versions, sensor firmware, lidar RPM and
   return mode.
4. Confirm the installed package's config is the workspace version. A stale
   `install/fast_lio/share/fast_lio/config` copy is a common source of false
   tuning conclusions.

### Stage B — validate topics without FAST-LIO

```bash
ros2 topic info /velodyne_points --verbose
ros2 topic info /camera/camera/imu --verbose
ros2 topic hz /velodyne_points
ros2 topic hz /camera/camera/imu
ros2 topic delay /velodyne_points
ros2 topic delay /camera/camera/imu
```

Programmatically inspect at least hundreds of messages:

- point field datatype and names;
- min/max/last/negative/nonfinite `time`;
- min/max/invalid `ring`;
- cloud header delta and point-derived duration;
- IMU stamp delta/gaps/backwards jumps;
- acceleration/gyro population, means, variances, and norms;
- cross-sensor offset stability;
- all of the above under full-stack load.

Do not rely only on `ros2 topic echo`; binary cloud data needs a small
`sensor_msgs_py.point_cloud2.read_points` audit or an equivalent C++ tool.

### Stage C — calibrate the D456

1. Warm up for a defined interval.
2. Run the official unit-specific IMU calibration.
3. Validate axes, signs, acceleration norm, stationary gyro, and rates.
4. Record long stationary bags with motors off and on.
5. Estimate white noise and bias random walk; select conservative scalar YAML
   values and document how they were converted.

### Stage D — calibrate space and time jointly

1. Set a valid point-time contract first.
2. Collect multiple calibration bags with rich rotations about x/y/z,
   translations, varying speeds, and 3-D structure.
3. Keep the rig rigid and avoid long degenerate tunnels for calibration.
4. Run LI-Init or an equivalent joint method.
5. Compare extrinsic and time offset across bags/cold starts.
6. Validate on held-out dynamic bags.
7. Freeze `extrinsic_est_en: false`.

### Stage E — tune geometry/computation

Use an ablation matrix, not simultaneous guesses:

1. `point_filter_num`: 1, then 2.
2. `filter_size_surf`: 0.10, 0.15, 0.20, 0.30 m.
3. `filter_size_map`: 0.15, 0.20, 0.25, 0.30, 0.50 m.
4. driver `max_range`: 30, 50, 70 m as environment permits.
5. `det_range`/`cube_side_length`: matched pairs such as 30/150, 40/200,
   50/250 m.
6. `max_iteration`: 3 vs 4 only after real-time margin exists.
7. single vs dual return only after the single-return baseline passes.

Score each run using the same bags and initial condition:

- absolute/relative trajectory error against ground truth when available;
- loop-closure endpoint error without applying loop closure;
- stationary position/orientation drift;
- repeated-run trajectory consistency;
- gravity alignment and estimated velocity at stops;
- residual/match statistics;
- CPU p50/p95/p99 update time, missed scan deadlines, memory, and topic loss;
- behavior in deliberately degenerate tunnel/ramp segments.

### Stage F — full-stack qualification

1. Enable only required publications.
2. Run perception, navigation, logging, cameras, and network traffic together.
3. Test cold start, warm start, reset, USB reconnect, packet loss, CPU load,
   temperature, vibration, dust, ramps/stairs, rapid yaw, and stops.
4. Establish explicit downstream rejection/failover thresholds.
5. Repeat long missions; do not qualify from one visually good RViz run.

## 9. Immediate recommended experiment

Make no code-quality tuning changes for the first comparison. Record one
representative raw bag, then replay the exact same bag through:

1. current Espeleo profile;
2. `timestamp_first_packet: true`,
   `extrinsic_est_en: false`, all optional publications off;
3. case 2 plus `point_filter_num: 1`, `blind: 0.9`,
   `filter_size_surf: 0.15`, `filter_size_map: 0.25`;
4. case 3 plus the jointly calibrated time offset;
5. case 4 plus measured D456 noise values.

This ordering separates timestamp/extrinsic failures from density/noise tuning.
If case 2 changes the result substantially, timing/online calibration—not the
sensor pairing—was the dominant problem.

## 10. Reliability decision

Treat VLP-16 + D456 FAST-LIO2 as production-ready only if all of these are true:

- point-time unit, sign, range, ordering, and scan-start header are enforced;
- cross-sensor offset is calibrated, stable across launches, and monitored;
- QoS/rates/gaps pass under full system load;
- IMU calibration, axes, frame, and initialization stationarity are proven;
- spatial extrinsics repeat across datasets and online estimation is disabled;
- tunnel/ramp degeneracy is detected and handled downstream;
- optional map/PCD/dense publication cannot starve the estimator;
- output frame and base-link lever arm are integrated correctly;
- a higher layer handles global drift;
- regression bags and quantitative acceptance thresholds are maintained.

If the time offset drifts materially, USB/UDP jitter is unbounded, or the
mission geometry is routinely unobservable with 16 beams, tuning alone will
not make it reliable. At that point the correct engineering change is a
hardware-synchronized IMU/lidar timestamp architecture, a denser lidar or
different mounting geometry, and/or complementary wheel/visual/global
constraints—not increasingly aggressive covariance or voxel tuning.

## Primary references

- [FAST-LIO upstream repository](https://github.com/hku-mars/FAST_LIO)
- [FAST-LIO2 paper](https://arxiv.org/abs/2107.06829)
- [LI-Init repository](https://github.com/hku-mars/LiDAR_IMU_Init)
- [LI-Init paper](https://arxiv.org/abs/2202.11006)
- [LI-Init Velodyne example configuration](https://raw.githubusercontent.com/hku-mars/LiDAR_IMU_Init/main/config/velodyne.yaml)
- [ROS 2 Velodyne driver source](https://raw.githubusercontent.com/ros-drivers/velodyne/ros2/velodyne_driver/src/driver/driver.cpp)
- [ROS 2 Velodyne packet timestamp source](https://raw.githubusercontent.com/ros-drivers/velodyne/ros2/velodyne_driver/src/lib/input.cpp)
- [ROS 2 Velodyne point conversion source](https://raw.githubusercontent.com/ros-drivers/velodyne/ros2/velodyne_pointcloud/src/lib/rawdata.cpp)
- [ROS 2 Velodyne pointcloud package](https://github.com/ros-drivers/velodyne/tree/ros2/velodyne_pointcloud)
- [VLP-16 user manual](https://data.ouster.io/downloads/velodyne/user-manual/vlp-16-user-manual-revf.pdf)
- [RealSense D456 product page](https://www.realsenseai.com/products/d456-usb/)
- [RealSense D400 series datasheet](https://dev.realsenseai.com/download/42003/)
- [RealSense ROS wrapper](https://github.com/realsenseai/realsense-ros)
- [librealsense D400 IMU documentation](https://raw.githubusercontent.com/IntelRealSense/librealsense/master/doc/d435i.md)
- [Bosch BMI085 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi085-ds001.pdf)
