# X5 ROS Driver

ROS driver for Insta360 X5 camera. Streams preview video and IMU data while recording full-resolution video to SD card.

## Features

- **Dual fisheye streaming**: Publishes both lens images at ~30Hz
- **High-rate IMU**: ~500Hz IMU data with timestamps
- **Automatic SD recording**: Full 8K recording starts/stops with the node
- **Clean shutdown**: Gracefully stops recording on Ctrl+C

## Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/x5/lens0` | `sensor_msgs/Image` | 30 Hz | Front fisheye (preview resolution) |
| `/x5/lens1` | `sensor_msgs/Image` | 30 Hz | Rear fisheye (preview resolution) |
| `/x5/imu` | `sensor_msgs/Imu` | ~500 Hz | IMU data (accel + gyro) |
| `/x5/exposure` | `sensor_msgs/TimeReference` | 30 Hz | Per-frame exposure timing |
| `/x5/battery_level` | `std_msgs/Int32` | 0.03 Hz | Battery percentage (0-100), latched |
| `/x5/recording_info` | `std_msgs/String` | Latched | SD card file path after recording |

## Installation

### Prerequisites

```bash
# ROS dependencies
sudo apt-get install ros-noetic-cv-bridge ros-noetic-image-transport

# FFmpeg
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libavutil-dev

# libusb
sudo apt-get install libusb-dev libudev-dev
```

### Insta360 SDK

1. Apply for SDK at https://www.insta360.com/sdk/apply
2. Extract to a known location (e.g., `~/CameraSDK`)

### Build

```bash
cd ~/catkin_ws/src
# Copy or link x5_ros_driver here

cd ~/catkin_ws
catkin build x5_ros_driver -DINSTA360_SDK_PATH=/path/to/CameraSDK
```

### USB Permissions (Important!)

The camera requires USB permissions. You have two options:

**Option A: Setup udev rules (Recommended)**
```bash
cd ~/catkin_ws/src/x5_ros_driver
chmod +x setup_udev.sh
./setup_udev.sh
# Reconnect camera and log out/in
```

**Option B: Run with sudo**
```bash
sudo -E bash -c 'source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && roslaunch x5_ros_driver x5_driver.launch'
```

## Usage

### Camera Setup

1. Connect X5 via USB
2. When prompted on camera, select **Android mode** (not USB storage)
3. Verify connection: `lsusb` should show device with ID `0x2e1a`

### Running

```bash
# Basic launch (auto-starts 8K recording)
roslaunch x5_ros_driver x5_driver.launch

# Custom preview resolution
roslaunch x5_ros_driver x5_driver.launch preview_resolution:=2k

# Without SD recording (streaming only)
roslaunch x5_ros_driver x5_driver.launch auto_start_recording:=false
```

### Recording Data

```bash
# Terminal 1: Start X5 driver
roslaunch x5_ros_driver x5_driver.launch

# Terminal 2: Record all data
rosbag record -o dataset \
    /x5/lens0 \
    /x5/lens1 \
    /x5/imu \
    /x5/exposure \
    /cv7/imu \
    /camera_trigger_time \
    /ouster/points

# When done: Ctrl+C both terminals
# Node will log: "SD Recording saved: /DCIM/Camera01/VID_xxx.insv"
```

### Downloading SD Card Recording

After recording, download the `.insv` file from the X5's SD card. You can:

1. Put camera in USB storage mode and copy files
2. Use the camera's WiFi transfer
3. Remove SD card and use a card reader

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `preview_resolution` | `"1080"` | Preview stream resolution: `4k`, `2k`, `1080`, `960` |
| `recording_resolution` | `"8k"` | SD recording resolution: `8k`, `5.7k` |
| `auto_start_recording` | `true` | Start SD recording on launch |
| `battery_poll_interval_sec` | `30` | How often to check/publish battery level |
| `low_battery_threshold_percent` | `15` | Auto-save and shutdown below this level |
| `lens0_frame_id` | `"x5_lens0"` | TF frame ID for front lens |
| `lens1_frame_id` | `"x5_lens1"` | TF frame ID for rear lens |
| `imu_frame_id` | `"x5_imu"` | TF frame ID for IMU |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/x5/shutdown` | `std_srvs/Trigger` | Powers off camera (enables charging via USB) |

## Battery Monitoring

The node monitors battery level and publishes to `/x5/battery_level` (std_msgs/Int32).

### Auto-Save on Low Battery

When battery drops below `low_battery_threshold_percent`:
1. Warning messages appear when battery is within 10% of threshold
2. At threshold: node auto-saves recording and shuts down cleanly
3. Your data is preserved - check the logged .insv file path

```bash
# Monitor battery in another terminal
rostopic echo /x5/battery_level

# Custom threshold (default 15%)
roslaunch x5_ros_driver x5_driver.launch low_battery_threshold_percent:=20
```

### Manual Shutdown

```bash
# Power off camera anytime (will charge via USB)
rosservice call /x5/shutdown

# Camera will power off
# Press power button on camera to restart
```

## Timestamps

All messages use X5's native timestamps (in seconds, converted from milliseconds).

- IMU and video share the same clock domain
- For synchronization with external sensors (CV7, LiDAR), use offline processing
- The `/x5/exposure` topic includes both ROS time (`header.stamp`) and X5 time (`time_ref`)

## Offline Processing

The live stream provides:
- Preview-quality images (for monitoring and sync reference)
- Full IMU data (same as embedded in .insv)

For final SLAM processing:
1. Extract full 4K+4K frames from `.insv` file
2. Cross-correlate IMU to align timestamps with CV7/LiDAR
3. Apply rolling shutter correction using sweep time (~6.75ms)
4. Select frames closest to LiDAR timestamps

## Troubleshooting

### "No cameras found"
- Check USB connection
- Ensure camera is in Android mode
- Run with `sudo` if permissions issue
- Check `lsusb` for device visibility

### "Failed to start streaming"
- Camera may be in wrong mode
- Try power cycling the camera
- Check if another app is using the camera

### Low frame rate
- Reduce `preview_resolution` to `960` or `1080`
- Check CPU usage
- Ensure USB 3.0 connection

### Corrupted recording
- Always stop node cleanly (Ctrl+C)
- Check SD card space before recording
- Check battery level

## Technical Details

- Preview stream: H.264 encoded, decoded via FFmpeg
- IMU rate: ~500Hz (delivered in batches)
- Rolling shutter: ~6.75ms sweep time
- Timestamps: milliseconds (X5 native clock)
- Accel units: m/s² (converted from g)
- Gyro units: rad/s

## License

MIT License
