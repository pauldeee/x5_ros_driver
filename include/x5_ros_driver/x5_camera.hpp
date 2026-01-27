#pragma once

#include <memory>
#include <functional>
#include <mutex>
#include <atomic>
#include <string>
#include <vector>
#include <deque>
#include <queue>
#include <thread>
#include <condition_variable>

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
 * @brief Frame filter callback - return true to decode frame, false to skip
 * Called before decoding to allow skipping frames (e.g., for trigger sync)
 * @param timestamp_ms Camera timestamp of the frame
 * @return true to decode this frame, false to skip it
 */
using FrameFilterCallback = std::function<bool(int64_t timestamp_ms)>;

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
     * @brief Set frame filter callback (called before decoding)
     * Use this to skip decoding frames that won't be needed (e.g., trigger sync mode)
     * If not set, all frames are decoded.
     */
    void setFrameFilter(FrameFilterCallback filter);

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

    /**
     * @brief Sync camera clock to system time and compute offset
     * @param max_offset_ms Maximum acceptable offset (not used for validation anymore)
     * @return true if offset computed successfully
     */
    bool syncTimeToSystem(int64_t max_offset_ms = 100);

    /**
     * @brief Get current camera media time
     * @return Camera time in milliseconds, or -1 on error
     */
    int64_t getCameraTime();

    /**
     * @brief Get time offset (system_time_ms = camera_time_ms + offset)
     */
    int64_t getTimeOffset() const { return time_offset_ms_.load(); }

    /**
     * @brief Check if camera time offset has been computed
     */
    bool isTimeOffsetValid() const { return time_offset_valid_; }

    /**
     * @brief Set GPS time offset (call when /ext/time received)
     * @param system_time_ms System/PC time in milliseconds
     * @param gps_time_ms GPS time in milliseconds
     */
    void setGpsTimeOffset(int64_t system_time_ms, int64_t gps_time_ms);

    /**
     * @brief Refresh camera→system time offset to track clock drift.
     * Call periodically (e.g., from GPS time callback) to prevent
     * the static offset from going stale as clocks drift.
     */
    void refreshTimeOffset();

    /**
     * @brief Check if GPS time offset is valid
     */
    bool isGpsTimeOffsetValid() const { return gps_time_offset_valid_; }

    /**
     * @brief Get GPS time offset (gps_time_ms = system_time_ms + offset)
     */
    int64_t getGpsTimeOffset() const { return gps_time_offset_ms_; }

    /**
     * @brief Get queue drop counts for diagnostics
     */
    uint64_t getPacketsDropped() const { return packets_dropped_.load(); }
    uint64_t getFramesDropped() const { return frames_dropped_.load(); }

    /**
     * @brief Convert camera timestamp to system/ROS time
     * @param camera_time_ms Camera media time in milliseconds
     * @return System time in milliseconds
     */
    int64_t cameraTimeToSystemTime(int64_t camera_time_ms);

    /**
     * @brief Convert camera timestamp to GPS time
     * @param camera_time_ms Camera media time in milliseconds
     * @return GPS time in milliseconds (or system time if GPS not available)
     */
    int64_t cameraTimeToGpsTime(int64_t camera_time_ms);

private:
    // SDK Stream delegate (inner class)
    class StreamDelegateImpl;
    friend class StreamDelegateImpl;

    // Time synchronization (camera to system)
    std::atomic<int64_t> time_offset_ms_{0};  // system_time_ms = camera_time_ms + offset
    bool time_offset_valid_ = false;

    // GPS time synchronization (system to GPS)
    int64_t gps_time_offset_ms_ = 0;      // gps_time_ms = system_time_ms + offset
    bool gps_time_offset_valid_ = false;
    double gps_offset_filtered_ = 0.0;
    bool gps_offset_initialized_ = false;
    std::mutex gps_time_mutex_;

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

    // Callbacks (separate mutexes to avoid blocking between IMU/video/exposure)
    ImuCallback imu_callback_;
    VideoCallback video_callback_;
    ExposureCallback exposure_callback_;
    FrameFilterCallback frame_filter_;  // Optional: skip frames before decoding
    std::mutex imu_callback_mutex_;
    std::mutex video_callback_mutex_;
    std::mutex exposure_callback_mutex_;
    std::mutex frame_filter_mutex_;

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

    // ========== Async Decoding Pipeline ==========
    // Stage 1: Encoded packet queue (SDK callback → decode thread)
    struct EncodedPacket {
        std::vector<uint8_t> data;
        int64_t timestamp;
        uint8_t stream_type;
        int stream_index;
    };

    std::queue<EncodedPacket> packet_queue_;
    std::mutex packet_queue_mutex_;
    std::condition_variable packet_queue_cv_;
    static constexpr size_t MAX_PACKET_QUEUE_SIZE = 5;

    // Stage 2: Decoded frame queue (decode thread → publish thread)
    struct DecodedFrame {
        cv::Mat image;          // Full decoded BGR frame
        int64_t timestamp;
        int stream_index;
    };

    std::queue<DecodedFrame> frame_queue_;
    std::mutex frame_queue_mutex_;
    std::condition_variable frame_queue_cv_;
    static constexpr size_t MAX_FRAME_QUEUE_SIZE = 4;  // 2 frames worth (lens0+lens1 each)

    // Drop counters for diagnostics
    std::atomic<uint64_t> packets_dropped_{0};
    std::atomic<uint64_t> frames_dropped_{0};

    // Worker threads
    std::thread decode_thread_;
    std::thread publish_thread_;
    std::atomic<bool> decode_thread_running_{false};
    std::atomic<bool> publish_thread_running_{false};

    void startDecodeThread();
    void stopDecodeThread();
    void decodeThreadLoop();
    void decodePacket(const EncodedPacket& packet);

    void startPublishThread();
    void stopPublishThread();
    void publishThreadLoop();
    void publishFrame(const DecodedFrame& frame);
};

} // namespace x5_ros_driver
