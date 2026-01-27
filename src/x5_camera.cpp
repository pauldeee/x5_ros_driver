#include "x5_ros_driver/x5_camera.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <ctime>


namespace x5_ros_driver {

// ============================================================================
// StreamDelegate Implementation (inner class)
// ============================================================================

class X5Camera::StreamDelegateImpl : public ins_camera::StreamDelegate {
public:
    StreamDelegateImpl(X5Camera* parent) : parent_(parent) {}

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {
        // Not used
    }

    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp,
                     uint8_t streamType, int stream_index) override {
        try {
            if (parent_) {
                parent_->onVideoData(data, size, timestamp, streamType, stream_index);
            }
        } catch (const std::exception& e) {
            std::cerr << "[X5Camera] OnVideoData exception: " << e.what() << std::endl;
        } catch (...) {}
    }

    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {
        try {
            if (parent_) {
                parent_->onImuData(data);
            }
        } catch (const std::exception& e) {
            std::cerr << "[X5Camera] OnGyroData exception: " << e.what() << std::endl;
        } catch (...) {}
    }

    void OnExposureData(const ins_camera::ExposureData& data) override {
        try {
            if (parent_) {
                parent_->onExposureData(data.timestamp, data.exposure_time);
            }
        } catch (const std::exception& e) {
            std::cerr << "[X5Camera] OnExposureData exception: " << e.what() << std::endl;
        } catch (...) {}
    }

private:
    X5Camera* parent_;
};

// ============================================================================
// X5Camera Implementation
// ============================================================================

X5Camera::X5Camera() {
    decoder_lens0_ = std::make_unique<VideoDecoder>();
    decoder_lens1_ = std::make_unique<VideoDecoder>();
}

X5Camera::~X5Camera() {
    // Stop decode thread first
    stopDecodeThread();

    if (streaming_) {
        stopStreaming();
    }
    disconnect();
}

bool X5Camera::connect() {
    if (connected_) {
        std::cout << "[X5Camera] Already connected" << std::endl;
        return true;
    }

    std::cout << "[X5Camera] Discovering cameras..." << std::endl;

    // Retry logic - the SDK sometimes fails on first attempt
    const int max_retries = 3;
    const int retry_delay_ms = 1000;
    
    ins_camera::DeviceDiscovery discovery;
    std::vector<ins_camera::DeviceDescriptor> devices;
    
    for (int attempt = 1; attempt <= max_retries; attempt++) {
        devices = discovery.GetAvailableDevices();
        
        if (!devices.empty()) {
            break;
        }
        
        if (attempt < max_retries) {
            std::cout << "[X5Camera] No cameras found, retrying in " 
                      << retry_delay_ms << "ms... (attempt " << attempt 
                      << "/" << max_retries << ")" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
        }
    }

    if (devices.empty()) {
        std::cerr << "[X5Camera] No cameras found after " << max_retries << " attempts!" << std::endl;
        std::cerr << "  Make sure:" << std::endl;
        std::cerr << "  1. X5 is connected via USB" << std::endl;
        std::cerr << "  2. X5 is in Android mode (not USB storage mode)" << std::endl;
        std::cerr << "  3. You have USB permissions (run with sudo or setup udev rules)" << std::endl;
        std::cerr << "  4. libusb is installed" << std::endl;
        std::cerr << "" << std::endl;
        std::cerr << "  To setup udev rules (avoids needing sudo):" << std::endl;
        std::cerr << "    sudo cp 99-insta360.rules /etc/udev/rules.d/" << std::endl;
        std::cerr << "    sudo udevadm control --reload-rules" << std::endl;
        std::cerr << "    sudo udevadm trigger" << std::endl;
        std::cerr << "    # Then reconnect the camera" << std::endl;
        return false;
    }

    // Find X5 (or use first camera)
    int selected_idx = 0;
    for (size_t i = 0; i < devices.size(); i++) {
        std::cout << "[X5Camera] Found: " << devices[i].camera_name 
                  << " (S/N: " << devices[i].serial_number << ")"
                  << " FW: " << devices[i].fw_version << std::endl;
        
        if (devices[i].camera_type == ins_camera::CameraType::Insta360X5) {
            selected_idx = i;
        }
    }

    auto& device = devices[selected_idx];
    serial_number_ = device.serial_number;
    firmware_version_ = device.fw_version;
    camera_name_ = device.camera_name;
    camera_type_ = device.camera_type;

    std::cout << "[X5Camera] Connecting to: " << camera_name_ << std::endl;

    // Create camera instance
    camera_ = std::make_shared<ins_camera::Camera>(device.info);

    // Create and set stream delegate
    stream_delegate_ = std::make_shared<StreamDelegateImpl>(this);
    std::shared_ptr<ins_camera::StreamDelegate> delegate = stream_delegate_;
    camera_->SetStreamDelegate(delegate);

    // Open camera
    if (!camera_->Open()) {
        std::cerr << "[X5Camera] Failed to open camera!" << std::endl;
        camera_.reset();
        discovery.FreeDeviceDescriptors(devices);
        return false;
    }

    discovery.FreeDeviceDescriptors(devices);
    connected_ = true;

    std::cout << "[X5Camera] Connected successfully!" << std::endl;
    return true;
}

void X5Camera::disconnect() {
    if (!connected_) {
        return;
    }

    std::cout << "[X5Camera] Disconnecting..." << std::endl;

    if (camera_) {
        camera_->Close();
        camera_.reset();
    }

    stream_delegate_.reset();
    connected_ = false;

    std::cout << "[X5Camera] Disconnected" << std::endl;
}

bool X5Camera::isConnected() const {
    return connected_ && camera_ && camera_->IsConnected();
}

ins_camera::VideoResolution X5Camera::toSdkPreviewResolution(PreviewResolution res) {
    switch (res) {
        case PreviewResolution::RES_960P:
            return ins_camera::VideoResolution::RES_1920_960P30;
        case PreviewResolution::RES_1080P:
            return ins_camera::VideoResolution::RES_2160_1080P30;
        case PreviewResolution::RES_2K:
            return ins_camera::VideoResolution::RES_2560_1280P30;
        case PreviewResolution::RES_4K:
            return ins_camera::VideoResolution::RES_3840_1920P30;
        default:
            return ins_camera::VideoResolution::RES_1920_960P30;
    }
}

ins_camera::VideoResolution X5Camera::toSdkRecordingResolution(RecordingResolution res) {
    switch (res) {
        case RecordingResolution::RES_5_7K:
            return ins_camera::VideoResolution::RES_57KP30;
        case RecordingResolution::RES_8K:
            return ins_camera::VideoResolution::RES_8KP30;
        default:
            return ins_camera::VideoResolution::RES_8KP30;
    }
}

bool X5Camera::startStreaming(PreviewResolution preview_res,
                               bool start_recording,
                               RecordingResolution recording_res,
                               bool use_hw_accel) {
    if (!connected_) {
        std::cerr << "[X5Camera] Not connected!" << std::endl;
        return false;
    }

    if (streaming_) {
        std::cerr << "[X5Camera] Already streaming!" << std::endl;
        return false;
    }

    auto sdk_preview_res = toSdkPreviewResolution(preview_res);
    auto sdk_recording_res = toSdkRecordingResolution(recording_res);

    // Start SD card recording FIRST if requested (before preview)
    if (start_recording) {
        std::cout << "[X5Camera] Starting SD card recording at " 
                  << (recording_res == RecordingResolution::RES_8K ? "8K" : "5.7K") 
                  << "..." << std::endl;

        // Set video mode and resolution for recording
        if (!camera_->SetVideoSubMode(ins_camera::SubVideoMode::VIDEO_NORMAL)) {
            std::cerr << "[X5Camera] Failed to set VIDEO_NORMAL mode for recording" << std::endl;
            return false;
        }

        ins_camera::RecordParams rec_params;
        rec_params.resolution = sdk_recording_res;
        rec_params.bitrate = 0;  // Use default bitrate
        if (!camera_->SetVideoCaptureParams(rec_params, 
                ins_camera::CameraFunctionMode::FUNCTION_MODE_NORMAL_VIDEO)) {
            std::cerr << "[X5Camera] Failed to set recording params" << std::endl;
            return false;
        }

        if (!camera_->StartRecording()) {
            std::cerr << "[X5Camera] Failed to start SD recording!" << std::endl;
            return false;
        }

        recording_ = true;
        std::cout << "[X5Camera] SD card recording started" << std::endl;
        
        // Give the camera a moment to stabilize
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Now setup preview streaming
    // For X4/X5, need to switch to liveview mode
    if (camera_type_ >= ins_camera::CameraType::Insta360X4) {
        std::cout << "[X5Camera] Setting up LiveView preview..." << std::endl;
        
        if (!camera_->SetVideoSubMode(ins_camera::SubVideoMode::VIDEO_LIVEVIEW)) {
            std::cerr << "[X5Camera] Failed to set VIDEO_LIVEVIEW mode" << std::endl;
            // Don't fail completely - recording might still work
            if (!start_recording) {
                return false;
            }
        }

        // Set preview resolution
        ins_camera::RecordParams preview_params;
        preview_params.resolution = sdk_preview_res;
        preview_params.bitrate = 0;
        if (!camera_->SetVideoCaptureParams(preview_params, 
                ins_camera::CameraFunctionMode::FUNCTION_MODE_LIVE_STREAM)) {
            std::cerr << "[X5Camera] Warning: Failed to set preview capture params" << std::endl;
        }
    }

    // Get preview parameters (includes sweep_time for rolling shutter)
    auto preview_params = camera_->GetPreviewParam();
    sweep_time_ns_ = preview_params.sweep_time;
    gyro_range_ = preview_params.gyro_range;
    accel_range_ = preview_params.acceleration_range;

    std::cout << "[X5Camera] Preview params:" << std::endl;
    std::cout << "  Sweep time (RS): " << sweep_time_ns_ << " ns (" 
              << (sweep_time_ns_ / 1e6) << " ms)" << std::endl;
    std::cout << "  Encode type: " 
              << (preview_params.encode_type == ins_camera::VideoEncodeType::H264 ? "H.264" : "H.265") 
              << std::endl;

    // Initialize video decoders
    VideoCodecType codec_type = (preview_params.encode_type == ins_camera::VideoEncodeType::H264) 
                                 ? VideoCodecType::H264 : VideoCodecType::H265;
    
    if (!decoder_lens0_->init(codec_type, use_hw_accel)) {
        std::cerr << "[X5Camera] Failed to initialize decoder for lens0" << std::endl;
        if (recording_) {
            camera_->StopRecording();
            recording_ = false;
        }
        return false;
    }

    if (!decoder_lens1_->init(codec_type, use_hw_accel)) {
        std::cerr << "[X5Camera] Failed to initialize decoder for lens1" << std::endl;
        if (recording_) {
            camera_->StopRecording();
            recording_ = false;
        }
        return false;
    }

    // Configure live stream
    ins_camera::LiveStreamParam stream_params;
    stream_params.video_resolution = sdk_preview_res;
    stream_params.lrv_video_resulution = ins_camera::VideoResolution::RES_1440_720P30;
    stream_params.video_bitrate = 1024 * 1024 * 2;  // 2 Mbps
    stream_params.enable_audio = false;
    stream_params.enable_gyro = true;  // Critical: enable IMU!
    stream_params.using_lrv = false;

    std::cout << "[X5Camera] Starting live stream..." << std::endl;

    if (!camera_->StartLiveStreaming(stream_params)) {
        std::cerr << "[X5Camera] Failed to start live streaming!" << std::endl;
        if (recording_) {
            camera_->StopRecording();
            recording_ = false;
        }
        return false;
    }

    // Start async decode thread
    startDecodeThread();

    streaming_ = true;
    std::cout << "[X5Camera] Live stream started!" << std::endl;

    return true;
}

std::vector<std::string> X5Camera::stopStreaming() {
    std::vector<std::string> recorded_files;

    if (!connected_) {
        return recorded_files;
    }

    // Stop async decode thread first
    stopDecodeThread();

    // Stop live streaming
    if (streaming_) {
        std::cout << "[X5Camera] Stopping live stream..." << std::endl;
        camera_->StopLiveStreaming();
        streaming_ = false;
    }

    // Stop SD recording
    if (recording_) {
        std::cout << "[X5Camera] Stopping SD recording..." << std::endl;
        
        auto media_url = camera_->StopRecording();
        recording_ = false;

        if (!media_url.Empty()) {
            for (const auto& url : media_url.OriginUrls()) {
                recorded_files.push_back(url);
                std::cout << "[X5Camera] Recorded file: " << url << std::endl;
            }
        } else {
            std::cerr << "[X5Camera] Warning: StopRecording returned empty URL" << std::endl;
        }
    }

    return recorded_files;
}

void X5Camera::setImuCallback(ImuCallback callback) {
    std::lock_guard<std::mutex> lock(imu_callback_mutex_);
    imu_callback_ = callback;
}

void X5Camera::setVideoCallback(VideoCallback callback) {
    std::lock_guard<std::mutex> lock(video_callback_mutex_);
    video_callback_ = callback;
}

void X5Camera::setExposureCallback(ExposureCallback callback) {
    std::lock_guard<std::mutex> lock(exposure_callback_mutex_);
    exposure_callback_ = callback;
}

void X5Camera::setFrameFilter(FrameFilterCallback filter) {
    std::lock_guard<std::mutex> lock(frame_filter_mutex_);
    frame_filter_ = filter;
}

void X5Camera::onImuData(const std::vector<ins_camera::GyroData>& data) {
    // Get callback pointer with minimal locking (don't block video processing)
    ImuCallback callback;
    {
        std::lock_guard<std::mutex> lock(imu_callback_mutex_);
        callback = imu_callback_;
    }

    if (!callback) {
        return;
    }

    // Process IMU data without holding any locks
    for (const auto& sample : data) {
        ImuSample imu;
        imu.timestamp_ms = static_cast<double>(sample.timestamp);

        // Scale accelerometer (g -> m/s²)
        imu.ax = sample.ax * accel_scale_;
        imu.ay = sample.ay * accel_scale_;
        imu.az = sample.az * accel_scale_;

        // Gyroscope (already rad/s, scale = 1.0)
        imu.gx = sample.gx * gyro_scale_;
        imu.gy = sample.gy * gyro_scale_;
        imu.gz = sample.gz * gyro_scale_;

        callback(imu);
    }
}

void X5Camera::onVideoData(const uint8_t* data, size_t size, int64_t timestamp,
                            uint8_t stream_type, int stream_index) {
    // Quick return if no callback or thread not running
    if (!video_callback_ || !decode_thread_running_) {
        return;
    }

    // Queue the encoded packet for async processing (minimal work in callback)
    {
        std::lock_guard<std::mutex> lock(packet_queue_mutex_);

        // Drop oldest packet if queue is full (prevents memory buildup)
        if (packet_queue_.size() >= MAX_PACKET_QUEUE_SIZE) {
            packet_queue_.pop();
            packets_dropped_++;
        }

        // Queue the packet (copy data to owned buffer)
        EncodedPacket packet;
        packet.data.assign(data, data + size);
        packet.timestamp = timestamp;
        packet.stream_type = stream_type;
        packet.stream_index = stream_index;
        packet_queue_.push(std::move(packet));
    }

    // Wake up worker thread
    packet_queue_cv_.notify_one();
}

void X5Camera::startDecodeThread() {
    if (decode_thread_running_) {
        return;
    }

    // Enable OpenCV parallel processing (uses all available cores for cvtColor, etc.)
    int num_threads = cv::getNumThreads();
    if (num_threads < 2) {
        cv::setNumThreads(4);  // Use up to 4 threads for OpenCV operations
    }
    std::cout << "[X5Camera] OpenCV using " << cv::getNumThreads() << " threads" << std::endl;

    decode_thread_running_ = true;
    decode_thread_ = std::thread(&X5Camera::decodeThreadLoop, this);
    std::cout << "[X5Camera] Decode thread started" << std::endl;

    // Also start publish thread (separate from decode for parallelism)
    startPublishThread();
}

void X5Camera::stopDecodeThread() {
    // Stop publish thread first (it consumes from frame queue)
    stopPublishThread();

    if (!decode_thread_running_) {
        return;
    }

    decode_thread_running_ = false;
    packet_queue_cv_.notify_all();  // Wake up thread to exit

    if (decode_thread_.joinable()) {
        decode_thread_.join();
    }
    std::cout << "[X5Camera] Decode thread stopped" << std::endl;

    // Clear any remaining packets
    {
        std::lock_guard<std::mutex> lock(packet_queue_mutex_);
        while (!packet_queue_.empty()) {
            packet_queue_.pop();
        }
    }

    // Clear any remaining decoded frames
    {
        std::lock_guard<std::mutex> lock(frame_queue_mutex_);
        while (!frame_queue_.empty()) {
            frame_queue_.pop();
        }
    }
}

void X5Camera::decodeThreadLoop() {
    std::cout << "[X5Camera] Decode thread running" << std::endl;

    while (decode_thread_running_) {
        EncodedPacket packet;

        // Wait for packet
        {
            std::unique_lock<std::mutex> lock(packet_queue_mutex_);
            packet_queue_cv_.wait(lock, [this] {
                return !packet_queue_.empty() || !decode_thread_running_;
            });

            if (!decode_thread_running_) {
                break;
            }

            if (packet_queue_.empty()) {
                continue;
            }

            packet = std::move(packet_queue_.front());
            packet_queue_.pop();
        }

        // Decode packet and queue result (no callback here - that's publish thread's job)
        try {
            decodePacket(packet);
        } catch (const std::exception& e) {
            std::cerr << "[X5Camera] Decode exception: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[X5Camera] Decode unknown exception" << std::endl;
        }
    }

    std::cout << "[X5Camera] Decode thread exiting" << std::endl;
}

void X5Camera::decodePacket(const EncodedPacket& packet) {
    // Always decode to maintain H.264 reference frame state.
    // Skipping packets breaks the P-frame decode chain, causing most frames
    // to fail decoding (only I-frames would succeed = ~1-2fps).
    VideoDecoder* decoder = decoder_lens0_.get();

    cv::Mat frame;
    if (!decoder->decode(packet.data.data(), packet.data.size(), frame)) {
        return;  // Decode failed (or no output for this NAL unit)
    }

    // Check frame filter AFTER decoding to decide whether to clone and queue.
    // This saves the expensive clone (~22MB per 4K frame) while keeping
    // VAAPI decode cost on the GPU.
    {
        std::lock_guard<std::mutex> lock(frame_filter_mutex_);
        if (frame_filter_) {
            if (!frame_filter_(packet.timestamp)) {
                return;  // Frame filtered out - skip clone and queue
            }
        }
    }

    // Queue decoded frame for publish thread
    // Must clone because decoder's buffer will be reused
    DecodedFrame df;
    df.image = frame.clone();
    df.timestamp = packet.timestamp;
    df.stream_index = packet.stream_index;

    {
        std::lock_guard<std::mutex> lock(frame_queue_mutex_);
        // Drop oldest if queue full (prevent memory buildup)
        if (frame_queue_.size() >= MAX_FRAME_QUEUE_SIZE) {
            frame_queue_.pop();
            frames_dropped_++;
        }
        frame_queue_.push(std::move(df));
    }
    frame_queue_cv_.notify_one();
}

// ========== Publish Thread ==========

void X5Camera::startPublishThread() {
    if (publish_thread_running_) {
        return;
    }

    publish_thread_running_ = true;
    publish_thread_ = std::thread(&X5Camera::publishThreadLoop, this);
    std::cout << "[X5Camera] Publish thread started" << std::endl;
}

void X5Camera::stopPublishThread() {
    if (!publish_thread_running_) {
        return;
    }

    publish_thread_running_ = false;
    frame_queue_cv_.notify_all();

    if (publish_thread_.joinable()) {
        publish_thread_.join();
    }
    std::cout << "[X5Camera] Publish thread stopped" << std::endl;

    // Clear any remaining frames
    std::lock_guard<std::mutex> lock(frame_queue_mutex_);
    while (!frame_queue_.empty()) {
        frame_queue_.pop();
    }
}

void X5Camera::publishThreadLoop() {
    std::cout << "[X5Camera] Publish thread running" << std::endl;

    while (publish_thread_running_) {
        DecodedFrame frame;

        // Wait for decoded frame
        {
            std::unique_lock<std::mutex> lock(frame_queue_mutex_);
            frame_queue_cv_.wait(lock, [this] {
                return !frame_queue_.empty() || !publish_thread_running_;
            });

            if (!publish_thread_running_) {
                break;
            }

            if (frame_queue_.empty()) {
                continue;
            }

            frame = std::move(frame_queue_.front());
            frame_queue_.pop();
        }

        // Process and publish frame (callback happens here)
        try {
            publishFrame(frame);
        } catch (const std::exception& e) {
            std::cerr << "[X5Camera] Publish callback exception: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[X5Camera] Publish callback unknown exception" << std::endl;
        }
    }

    std::cout << "[X5Camera] Publish thread exiting" << std::endl;
}

void X5Camera::publishFrame(const DecodedFrame& frame) {
    // Get callback pointer
    VideoCallback callback;
    {
        std::lock_guard<std::mutex> lock(video_callback_mutex_);
        callback = video_callback_;
    }

    if (!callback) {
        return;
    }

    const cv::Mat& img = frame.image;

    // Check if this is a dual fisheye frame (width ~= 2x height)
    bool is_dual_fisheye = (img.cols > img.rows * 1.5);

    if (is_dual_fisheye) {
        // Split into left and right halves
        int half_width = img.cols / 2;
        int height = img.rows;

        // Crop H.264 padding (height is often padded to multiple of 16)
        int target_height = height;
        if (height == 968) target_height = 960;
        else if (height == 1088) target_height = 1080;
        else if (height == 1296) target_height = 1280;
        else if (height == 1936) target_height = 1920;

        // Extract left fisheye (lens0)
        cv::Mat left_roi = img(cv::Rect(0, 0, half_width, target_height));
        VideoFrame vf0;
        vf0.timestamp_ms = static_cast<double>(frame.timestamp);
        vf0.stream_index = 0;
        vf0.image = left_roi;
        callback(vf0);

        // Extract right fisheye (lens1)
        cv::Mat right_roi = img(cv::Rect(half_width, 0, half_width, target_height));
        VideoFrame vf1;
        vf1.timestamp_ms = static_cast<double>(frame.timestamp);
        vf1.stream_index = 1;
        vf1.image = right_roi;
        callback(vf1);
    } else {
        // Single image
        int target_height = img.rows;
        if (img.rows == 968) target_height = 960;
        else if (img.rows == 1088) target_height = 1080;
        else if (img.rows == 1296) target_height = 1280;
        else if (img.rows == 1936) target_height = 1920;

        cv::Mat cropped = img;
        if (target_height != img.rows) {
            cropped = img(cv::Rect(0, 0, img.cols, target_height));
        }

        VideoFrame vf;
        vf.timestamp_ms = static_cast<double>(frame.timestamp);
        vf.stream_index = frame.stream_index;
        vf.image = cropped;
        callback(vf);
    }
}

void X5Camera::onExposureData(double timestamp, double exposure_time) {
    // Get callback pointer with minimal locking
    ExposureCallback callback;
    {
        std::lock_guard<std::mutex> lock(exposure_callback_mutex_);
        callback = exposure_callback_;
    }

    if (!callback) {
        return;
    }

    ExposureInfo exp;
    exp.timestamp_ms = timestamp;
    exp.exposure_time_sec = exposure_time;

    callback(exp);
}

int X5Camera::getBatteryLevel() {
    if (!connected_ || !camera_) {
        return -1;
    }

    ins_camera::BatteryStatus status;
    if (camera_->GetBatteryStatus(status)) {
        return static_cast<int>(status.battery_level);
    }
    return -1;
}

bool X5Camera::getStorageInfo(uint64_t& free_space_mb, uint64_t& total_space_mb) {
    if (!connected_ || !camera_) {
        return false;
    }

    ins_camera::StorageStatus status;
    if (camera_->GetStorageState(status)) {
        free_space_mb = status.free_space / (1024 * 1024);
        total_space_mb = status.total_space / (1024 * 1024);
        return true;
    }
    return false;
}

bool X5Camera::shutdownCamera() {
    if (!connected_ || !camera_) {
        std::cerr << "[X5Camera] Cannot shutdown - not connected" << std::endl;
        return false;
    }

    // Make sure streaming and recording are stopped first
    if (streaming_ || recording_) {
        std::cout << "[X5Camera] Stopping streaming/recording before shutdown..." << std::endl;
        stopStreaming();
    }

    std::cout << "[X5Camera] Sending shutdown command to camera..." << std::endl;
    
    bool result = camera_->ShutdownCamera();
    
    if (result) {
        std::cout << "[X5Camera] Shutdown command sent successfully" << std::endl;
        std::cout << "[X5Camera] Camera will power off and charge via USB" << std::endl;
        std::cout << "[X5Camera] Press power button on camera to turn it back on" << std::endl;
        
        // Camera is shutting down, mark as disconnected
        connected_ = false;
        camera_.reset();
    } else {
        std::cerr << "[X5Camera] Shutdown command failed!" << std::endl;
    }
    
    return result;
}

int64_t X5Camera::getCameraTime() {
    if (!connected_ || !camera_) {
        return -1;
    }
    return camera_->GetCameraMediaTime();
}

bool X5Camera::syncTimeToSystem(int64_t max_offset_ms) {
    if (!connected_ || !camera_) {
        std::cerr << "[X5Camera] Cannot sync time - not connected" << std::endl;
        return false;
    }

    // =========================================
    // Step 1: Sync camera wall clock (for file naming, etc.)
    // This doesn't affect GetCameraMediaTime()
    // =========================================
    time_t now = time(nullptr);
    std::tm tm{};
    localtime_r(&now, &tm);
    time_t time_seconds = timegm(&tm);

    std::cout << "[X5Camera] Syncing camera wall clock..." << std::endl;
    std::cout << "  Local time: " << now << std::endl;
    std::cout << "  GMT time: " << time_seconds << std::endl;
    std::cout << "  TZ offset: " << (time_seconds - now) << " sec" << std::endl;

    if (!camera_->SyncLocalTimeToCamera(now, time_seconds - now)) {
        std::cerr << "[X5Camera] SyncLocalTimeToCamera() failed (non-critical)" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // =========================================
    // Step 2: Compute offset between system time and camera media time
    // GetCameraMediaTime() returns time since boot, not wall clock
    // =========================================
    const int num_samples = 5;
    int64_t offset_sum = 0;

    std::cout << "[X5Camera] Computing camera→system time offset..." << std::endl;

    for (int i = 0; i < num_samples; i++) {
        auto t1 = std::chrono::system_clock::now();
        int64_t camera_time = camera_->GetCameraMediaTime();
        auto t2 = std::chrono::system_clock::now();

        // Use midpoint for best estimate
        auto t_mid = t1 + (t2 - t1) / 2;
        int64_t system_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            t_mid.time_since_epoch()).count();

        int64_t offset = system_time_ms - camera_time;
        offset_sum += offset;

        std::cout << "  Sample " << (i + 1) << ": system=" << system_time_ms
                  << " camera=" << camera_time << " offset=" << offset << " ms" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    time_offset_ms_.store(offset_sum / num_samples);
    time_offset_valid_ = true;

    std::cout << "[X5Camera] Camera→System offset: " << time_offset_ms_.load() << " ms" << std::endl;

    return true;
}

void X5Camera::refreshTimeOffset() {
    if (!connected_ || !camera_) {
        return;
    }

    auto t1 = std::chrono::system_clock::now();
    int64_t camera_time = camera_->GetCameraMediaTime();
    auto t2 = std::chrono::system_clock::now();

    auto t_mid = t1 + (t2 - t1) / 2;
    int64_t system_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        t_mid.time_since_epoch()).count();

    int64_t new_offset = system_time_ms - camera_time;

    // Use exponential moving average to smooth jitter while tracking drift
    int64_t old_offset = time_offset_ms_.load();
    if (time_offset_valid_) {
        const double alpha = 0.2;  // Faster than GPS filter to track drift promptly
        double filtered = alpha * new_offset + (1.0 - alpha) * static_cast<double>(old_offset);
        time_offset_ms_.store(static_cast<int64_t>(filtered));
    } else {
        time_offset_ms_.store(new_offset);
        time_offset_valid_ = true;
    }
}

void X5Camera::setGpsTimeOffset(int64_t system_time_ms, int64_t gps_time_ms) {
    std::lock_guard<std::mutex> lock(gps_time_mutex_);

    int64_t new_offset = gps_time_ms - system_time_ms;

    if (!gps_offset_initialized_) {
        // First sample - initialize directly
        gps_offset_filtered_ = static_cast<double>(new_offset);
        gps_offset_initialized_ = true;
        std::cout << "[X5Camera] GPS time offset initialized: " << new_offset << " ms" << std::endl;
    } else {
        // Low-pass filter: alpha = 0.1 gives ~10 sample smoothing
        const double alpha = 0.1;
        gps_offset_filtered_ = alpha * new_offset + (1.0 - alpha) * gps_offset_filtered_;
    }

    gps_time_offset_ms_ = static_cast<int64_t>(gps_offset_filtered_);
    gps_time_offset_valid_ = true;
}

int64_t X5Camera::cameraTimeToSystemTime(int64_t camera_time_ms) {
    return camera_time_ms + time_offset_ms_.load();
}

int64_t X5Camera::cameraTimeToGpsTime(int64_t camera_time_ms) {
    // Step 1: camera → system
    int64_t system_time_ms = camera_time_ms + time_offset_ms_.load();

    // Step 2: system → GPS (if available)
    std::lock_guard<std::mutex> lock(gps_time_mutex_);
    if (gps_time_offset_valid_) {
        return system_time_ms + gps_time_offset_ms_;
    }
    return system_time_ms;  // Fallback to system time
}

} // namespace x5_ros_driver
