#pragma once

#include <memory>
#include <functional>
#include <mutex>
#include <atomic>
#include <string>
#include <vector>
#include <deque>

#include <camera/camera.h>
#include <camera/device_discovery.h>
#include <camera/photography_settings.h>

#include "x5_ros_driver/video_decoder.hpp"

namespace x5_ros_driver {

/**
 * @brief IMU sample from X5
 */
struct ImuSample {
    double timestamp_ms;    // X5 timestamp in milliseconds
    double ax, ay, az;      // Accelerometer (g)
    double gx, gy, gz;      // Gyroscope (rad/s)
};

/**
 * @brief Decoded video frame from X5
 */
struct VideoFrame {
    double timestamp_ms;    // X5 timestamp in milliseconds
    int stream_index;       // 0 = lens0, 1 = lens1
    cv::Mat image;          // Decoded BGR image
};

/**
 * @brief Exposure data from X5
 */
struct ExposureInfo {
    double timestamp_ms;
    double exposure_time_sec;
};

/**
 * @brief Callback types for async data delivery
 */
using ImuCallback = std::function<void(const ImuSample&)>;
using VideoCallback = std::function<void(const VideoFrame&)>;
using ExposureCallback = std::function<void(const ExposureInfo&)>;

/**
 * @brief Resolution options for preview stream
 */
enum class PreviewResolution {
    RES_960P,   // 1920x960
    RES_1080P,  // 2160x1080
    RES_2K,     // 2560x1280
    RES_4K      // 3840x1920
};

/**
 * @brief Resolution options for SD recording
 */
enum class RecordingResolution {
    RES_5_7K,   // 5.7K
    RES_8K      // 8K (full quality)
};

/**
 * @brief X5 Camera wrapper class
 * 
 * Handles connection, streaming, and SD card recording for the Insta360 X5.
 * Implements the SDK's StreamDelegate interface internally.
 */
class X5Camera {
public:
    X5Camera();
    ~X5Camera();

    // Disable copy
    X5Camera(const X5Camera&) = delete;
    X5Camera& operator=(const X5Camera&) = delete;

    /**
     * @brief Discover and connect to X5 camera
     * @return true if connected successfully
     */
    bool connect();

    /**
     * @brief Disconnect from camera
     */
    void disconnect();

    /**
     * @brief Check if connected
     */
    bool isConnected() const;

    /**
     * @brief Start live preview stream (and optionally SD recording)
     * @param preview_res Resolution for preview stream
     * @param start_recording If true, also start SD card recording
     * @param recording_res Resolution for SD recording
     * @return true if started successfully
     */
    bool startStreaming(PreviewResolution preview_res = PreviewResolution::RES_1080P,
                        bool start_recording = true,
                        RecordingResolution recording_res = RecordingResolution::RES_8K);

    /**
     * @brief Stop streaming and recording
     * @return Vector of recorded file paths on SD card
     */
    std::vector<std::string> stopStreaming();

    /**
     * @brief Check if currently streaming
     */
    bool isStreaming() const { return streaming_.load(); }

    /**
     * @brief Check if currently recording to SD
     */
    bool isRecording() const { return recording_.load(); }

    /**
     * @brief Set callback for IMU data
     */
    void setImuCallback(ImuCallback callback);

    /**
     * @brief Set callback for video frames
     */
    void setVideoCallback(VideoCallback callback);

    /**
     * @brief Set callback for exposure data
     */
    void setExposureCallback(ExposureCallback callback);

    /**
     * @brief Get camera info
     */
    std::string getSerialNumber() const { return serial_number_; }
    std::string getFirmwareVersion() const { return firmware_version_; }
    std::string getCameraName() const { return camera_name_; }

    /**
     * @brief Get rolling shutter sweep time
     * @return Sweep time in nanoseconds
     */
    int64_t getSweepTimeNs() const { return sweep_time_ns_; }

    /**
     * @brief Get battery level
     * @return Battery percentage (0-100)
     */
    int getBatteryLevel();

    /**
     * @brief Get storage info
     * @param free_space_mb Output: free space in MB
     * @param total_space_mb Output: total space in MB
     * @return true if successful
     */
    bool getStorageInfo(uint64_t& free_space_mb, uint64_t& total_space_mb);

    /**
     * @brief Shutdown the camera (powers off)
     * Camera will turn off and start charging if connected to USB power.
     * You'll need to physically press the power button to turn it back on.
     * @return true if shutdown command was sent successfully
     */
    bool shutdownCamera();

private:
    // SDK Stream delegate (inner class)
    class StreamDelegateImpl;
    friend class StreamDelegateImpl;

    // Camera instance
    std::shared_ptr<ins_camera::Camera> camera_;
    std::shared_ptr<StreamDelegateImpl> stream_delegate_;

    // Video decoders (one per lens)
    std::unique_ptr<VideoDecoder> decoder_lens0_;
    std::unique_ptr<VideoDecoder> decoder_lens1_;

    // Camera info
    std::string serial_number_;
    std::string firmware_version_;
    std::string camera_name_;
    ins_camera::CameraType camera_type_;

    // Preview parameters
    int64_t sweep_time_ns_ = 0;
    int32_t gyro_range_ = 0;
    int32_t accel_range_ = 0;

    // State
    std::atomic<bool> connected_{false};
    std::atomic<bool> streaming_{false};
    std::atomic<bool> recording_{false};

    // Callbacks
    ImuCallback imu_callback_;
    VideoCallback video_callback_;
    ExposureCallback exposure_callback_;
    std::mutex callback_mutex_;

    // IMU scaling (from diagnostic: accel in g, gyro in rad/s)
    double accel_scale_ = 9.81;  // Convert g to m/s²
    double gyro_scale_ = 1.0;    // Already rad/s

    // Internal methods
    void onImuData(const std::vector<ins_camera::GyroData>& data);
    void onVideoData(const uint8_t* data, size_t size, int64_t timestamp, 
                     uint8_t stream_type, int stream_index);
    void onExposureData(double timestamp, double exposure_time);

    ins_camera::VideoResolution toSdkPreviewResolution(PreviewResolution res);
    ins_camera::VideoResolution toSdkRecordingResolution(RecordingResolution res);
};

} // namespace x5_ros_driver
