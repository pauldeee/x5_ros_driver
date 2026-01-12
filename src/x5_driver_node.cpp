/**
 * X5 ROS Driver Node
 * 
 * Streams video and IMU data from Insta360 X5 camera while recording
 * full-resolution video to SD card.
 * 
 * Published Topics:
 *   /x5/lens0     - Front fisheye image (sensor_msgs/Image)
 *   /x5/lens1     - Rear fisheye image (sensor_msgs/Image)
 *   /x5/imu       - IMU data at ~500Hz (sensor_msgs/Imu)
 *   /x5/exposure  - Per-frame exposure info (sensor_msgs/TimeReference)
 * 
 * The node automatically starts SD card recording on startup and stops
 * on shutdown, logging the recorded file path.
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Trigger.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <csignal>
#include <atomic>
#include <mutex>

#include "x5_ros_driver/x5_camera.hpp"

using namespace x5_ros_driver;

// ============================================================================
// Global state for signal handling
// ============================================================================

std::atomic<bool> g_shutdown_requested{false};
std::unique_ptr<X5Camera> g_camera;

void signalHandler(int sig) {
    ROS_WARN("Received signal %d, initiating clean shutdown...", sig);
    g_shutdown_requested = true;
    ros::shutdown();
}

// ============================================================================
// ROS Node Class
// ============================================================================

class X5DriverNode {
public:
    X5DriverNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
        : nh_(nh), pnh_(pnh), it_(nh) {
        
        // Load parameters
        loadParameters();

        // Create publishers
        pub_lens0_ = it_.advertise("x5/lens0", 1);
        pub_lens1_ = it_.advertise("x5/lens1", 1);
        pub_combined_ = it_.advertise("x5/combined", 1);
        pub_imu_ = nh_.advertise<sensor_msgs::Imu>("x5/imu", 100);
        pub_exposure_ = nh_.advertise<sensor_msgs::TimeReference>("x5/exposure", 10);
        pub_recording_info_ = nh_.advertise<std_msgs::String>("x5/recording_info", 1, true);  // Latched
        pub_battery_ = nh_.advertise<std_msgs::Int32>("x5/battery_level", 1, true);  // Latched

        // Create services
        srv_shutdown_ = nh_.advertiseService("x5/shutdown", &X5DriverNode::shutdownCallback, this);

        ROS_INFO("X5 Driver Node initialized");
        ROS_INFO("  Service /x5/shutdown - Powers off camera (will charge via USB)");
        ROS_INFO("  Topic /x5/battery_level - Battery percentage (polled every %d sec)", battery_poll_interval_sec_);
    }

    ~X5DriverNode() {
        shutdown();
    }

    bool start() {
        // Create camera
        camera_ = std::make_unique<X5Camera>();
        g_camera = std::move(camera_);
        camera_ = nullptr;  // Use g_camera from now on

        // Connect to camera
        if (!g_camera->connect()) {
            ROS_ERROR("Failed to connect to X5 camera");
            return false;
        }

        // Set callbacks
        g_camera->setImuCallback([this](const ImuSample& imu) {
            publishImu(imu);
        });

        g_camera->setVideoCallback([this](const VideoFrame& frame) {
            publishVideo(frame);
        });

        g_camera->setExposureCallback([this](const ExposureInfo& exp) {
            publishExposure(exp);
        });

        // Get storage info
        uint64_t free_mb, total_mb;
        if (g_camera->getStorageInfo(free_mb, total_mb)) {
            ROS_INFO("SD Card: %lu MB free / %lu MB total", free_mb, total_mb);
            if (free_mb < 1024) {
                ROS_WARN("Low storage! Only %lu MB free on SD card", free_mb);
            }
        }

        // Get battery
        int battery = g_camera->getBatteryLevel();
        if (battery >= 0) {
            ROS_INFO("Battery: %d%%", battery);
            if (battery < 20) {
                ROS_WARN("Low battery! %d%%", battery);
            }
        }

        // Start streaming + recording
        PreviewResolution preview_res = PreviewResolution::RES_1080P;
        if (preview_resolution_ == "4k") {
            preview_res = PreviewResolution::RES_4K;
        } else if (preview_resolution_ == "2k") {
            preview_res = PreviewResolution::RES_2K;
        } else if (preview_resolution_ == "960") {
            preview_res = PreviewResolution::RES_960P;
        }

        RecordingResolution rec_res = RecordingResolution::RES_8K;
        if (recording_resolution_ == "5.7k") {
            rec_res = RecordingResolution::RES_5_7K;
        }

        if (!g_camera->startStreaming(preview_res, auto_start_recording_, rec_res)) {
            ROS_ERROR("Failed to start streaming");
            return false;
        }

        // Log sweep time
        int64_t sweep_ns = g_camera->getSweepTimeNs();
        ROS_INFO("Rolling shutter sweep time: %.3f ms", sweep_ns / 1e6);

        running_ = true;
        ROS_INFO("X5 streaming started!");
        
        if (auto_start_recording_) {
            ROS_INFO("SD card recording started (8K)");
            ROS_INFO("Recording will be saved when node is stopped");
        }

        // Start battery monitoring timer
        battery_timer_ = nh_.createTimer(
            ros::Duration(battery_poll_interval_sec_),
            &X5DriverNode::batteryTimerCallback, this);
        
        // Publish initial battery level
        publishBatteryLevel();

        return true;
    }

    void spin() {
        ros::Rate rate(100);  // 100 Hz check rate
        
        while (ros::ok() && !g_shutdown_requested && running_) {
            ros::spinOnce();
            rate.sleep();

            // Periodic connection check
            static int counter = 0;
            if (++counter >= 1000) {  // Every ~10 seconds
                counter = 0;
                if (!g_camera || !g_camera->isConnected()) {
                    ROS_ERROR("Camera disconnected!");
                    running_ = false;
                }
            }
        }
    }

    void shutdown() {
        if (!running_) {
            return;
        }

        running_ = false;
        ROS_INFO("Shutting down X5 driver...");

        if (g_camera) {
            auto recorded_files = g_camera->stopStreaming();
            
            // Publish recording info
            if (!recorded_files.empty()) {
                std_msgs::String msg;
                msg.data = "Recorded files:\n";
                for (const auto& file : recorded_files) {
                    msg.data += "  " + file + "\n";
                    ROS_INFO("SD Recording saved: %s", file.c_str());
                }
                pub_recording_info_.publish(msg);
            }

            g_camera->disconnect();
            g_camera.reset();
        }

        ROS_INFO("X5 driver shutdown complete");
    }

private:
    void loadParameters() {
        pnh_.param<std::string>("preview_resolution", preview_resolution_, "1080");
        pnh_.param<std::string>("recording_resolution", recording_resolution_, "8k");
        pnh_.param<bool>("auto_start_recording", auto_start_recording_, true);
        pnh_.param<int>("battery_poll_interval_sec", battery_poll_interval_sec_, 30);
        pnh_.param<int>("low_battery_threshold_percent", low_battery_threshold_percent_, 15);
        pnh_.param<std::string>("lens0_frame_id", lens0_frame_id_, "x5_lens0");
        pnh_.param<std::string>("lens1_frame_id", lens1_frame_id_, "x5_lens1");
        pnh_.param<std::string>("imu_frame_id", imu_frame_id_, "x5_imu");
        pnh_.param<bool>("publish_combined", publish_combined_, true);
        pnh_.param<std::string>("combined_format", combined_format_, "sidebyside");

        ROS_INFO("Parameters:");
        ROS_INFO("  preview_resolution: %s", preview_resolution_.c_str());
        ROS_INFO("  recording_resolution: %s", recording_resolution_.c_str());
        ROS_INFO("  auto_start_recording: %s", auto_start_recording_ ? "true" : "false");
        ROS_INFO("  battery_poll_interval_sec: %d", battery_poll_interval_sec_);
        ROS_INFO("  low_battery_threshold_percent: %d%%", low_battery_threshold_percent_);
    }

    void publishImu(const ImuSample& imu) {
        sensor_msgs::Imu msg;
        
        // Convert X5 timestamp (ms) to ROS time
        // For now, use X5 native time. Offline processing will handle sync.
        msg.header.stamp = ros::Time(imu.timestamp_ms / 1000.0);
        msg.header.frame_id = imu_frame_id_;

        // Angular velocity (rad/s)
        msg.angular_velocity.x = imu.gx;
        msg.angular_velocity.y = imu.gy;
        msg.angular_velocity.z = imu.gz;

        // Linear acceleration (m/s²)
        msg.linear_acceleration.x = imu.ax;
        msg.linear_acceleration.y = imu.ay;
        msg.linear_acceleration.z = imu.az;

        // Covariance unknown
        msg.orientation_covariance[0] = -1;  // Orientation not provided
        msg.angular_velocity_covariance[0] = 0;
        msg.linear_acceleration_covariance[0] = 0;

        pub_imu_.publish(msg);
        imu_count_++;
    }

    void publishVideo(const VideoFrame& frame) {
        if (frame.image.empty()) {
            return;
        }

        // Convert X5 timestamp (ms) to ROS time
        ros::Time stamp(frame.timestamp_ms / 1000.0);

        // Create image message
        std_msgs::Header header;
        header.stamp = stamp;
        header.frame_id = (frame.stream_index == 0) ? lens0_frame_id_ : lens1_frame_id_;

        cv_bridge::CvImage cv_img(header, "bgr8", frame.image);
        sensor_msgs::ImagePtr img_msg = cv_img.toImageMsg();

        // Publish to appropriate topic and store for combined
        if (frame.stream_index == 0) {
            pub_lens0_.publish(img_msg);
            lens0_count_++;

            if (publish_combined_) {
                std::lock_guard<std::mutex> lock(frame_sync_mutex_);
                last_lens0_frame_ = frame.image.clone();
                last_lens0_timestamp_ = frame.timestamp_ms;
            }
        } else {
            pub_lens1_.publish(img_msg);
            lens1_count_++;

            if (publish_combined_) {
                std::lock_guard<std::mutex> lock(frame_sync_mutex_);
                last_lens1_frame_ = frame.image.clone();
                last_lens1_timestamp_ = frame.timestamp_ms;
            }
        }

        // Try to publish combined image
        if (publish_combined_) {
            tryPublishCombined();
        }

        // Periodic logging
        static int log_counter = 0;
        if (++log_counter >= 300) {
            log_counter = 0;
            ROS_INFO("Published - lens0: %lu, lens1: %lu, imu: %lu", 
                     lens0_count_, lens1_count_, imu_count_);
        }
    }

    void tryPublishCombined() {
        std::lock_guard<std::mutex> lock(frame_sync_mutex_);

        // Need both frames
        if (last_lens0_frame_.empty() || last_lens1_frame_.empty()) {
            return;
        }

        // Check timestamps match (within 1ms - should be same frame)
        if (std::abs(last_lens0_timestamp_ - last_lens1_timestamp_) > 1.0) {
            return;
        }

        // Validate dimensions match
        if (last_lens0_frame_.size() != last_lens1_frame_.size()) {
            ROS_WARN_THROTTLE(5, "Lens frames have different sizes");
            return;
        }

        // Combine frames
        cv::Mat combined;
        if (combined_format_ == "topbottom") {
            cv::vconcat(last_lens0_frame_, last_lens1_frame_, combined);
        } else {
            // Default: sidebyside
            cv::hconcat(last_lens0_frame_, last_lens1_frame_, combined);
        }

        // Publish
        std_msgs::Header header;
        header.stamp = ros::Time(last_lens0_timestamp_ / 1000.0);
        header.frame_id = "x5_combined";

        cv_bridge::CvImage cv_img(header, "bgr8", combined);
        pub_combined_.publish(cv_img.toImageMsg());

        // Clear frames
        last_lens0_frame_.release();
        last_lens1_frame_.release();
    }

    void publishExposure(const ExposureInfo& exp) {
        sensor_msgs::TimeReference msg;
        
        // header.stamp = ROS time when we received this
        // time_ref = X5 timestamp
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = lens0_frame_id_;  // Associated with frames
        
        // Store X5 timestamp in time_ref
        msg.time_ref = ros::Time(exp.timestamp_ms / 1000.0);
        
        // Store exposure time in source field as string (hacky but simple)
        msg.source = std::to_string(exp.exposure_time_sec);

        pub_exposure_.publish(msg);
    }

    void publishBatteryLevel() {
        if (!g_camera || !g_camera->isConnected()) {
            return;
        }

        int battery = g_camera->getBatteryLevel();
        if (battery < 0) {
            ROS_WARN_THROTTLE(60, "Failed to get battery level");
            return;
        }

        // Publish battery level
        std_msgs::Int32 msg;
        msg.data = battery;
        pub_battery_.publish(msg);

        // Check for low battery
        if (battery <= low_battery_threshold_percent_ && !low_battery_triggered_) {
            low_battery_triggered_ = true;
            
            ROS_ERROR("===========================================");
            ROS_ERROR("  LOW BATTERY: %d%% (threshold: %d%%)", battery, low_battery_threshold_percent_);
            ROS_ERROR("  AUTO-SAVING RECORDING AND SHUTTING DOWN!");
            ROS_ERROR("===========================================");

            // Stop cleanly - this will save the recording
            running_ = false;
            ros::shutdown();
        } else if (battery <= low_battery_threshold_percent_ + 10) {
            ROS_WARN_THROTTLE(30, "Battery low: %d%% (will auto-save at %d%%)", 
                              battery, low_battery_threshold_percent_);
        }
    }

    void batteryTimerCallback(const ros::TimerEvent& event) {
        publishBatteryLevel();
    }

    bool shutdownCallback(std_srvs::Trigger::Request& req, 
                          std_srvs::Trigger::Response& res) {
        ROS_INFO("Shutdown service called");
        
        if (!g_camera) {
            res.success = false;
            res.message = "Camera not connected";
            return true;
        }

        // Stop streaming/recording first
        if (g_camera->isStreaming() || g_camera->isRecording()) {
            ROS_INFO("Stopping streaming/recording before shutdown...");
            auto recorded_files = g_camera->stopStreaming();
            
            if (!recorded_files.empty()) {
                for (const auto& file : recorded_files) {
                    ROS_INFO("SD Recording saved: %s", file.c_str());
                }
            }
        }

        // Send shutdown command
        if (g_camera->shutdownCamera()) {
            res.success = true;
            res.message = "Camera shutdown initiated. Press power button to restart.";
            ROS_INFO("Camera shutdown successful - will charge via USB");
            
            // Stop the node since camera is off
            running_ = false;
            ros::shutdown();
        } else {
            res.success = false;
            res.message = "Shutdown command failed";
            ROS_ERROR("Camera shutdown failed!");
        }

        return true;
    }

    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;

    // Publishers
    image_transport::Publisher pub_lens0_;
    image_transport::Publisher pub_lens1_;
    image_transport::Publisher pub_combined_;
    ros::Publisher pub_imu_;
    ros::Publisher pub_exposure_;
    ros::Publisher pub_recording_info_;
    ros::Publisher pub_battery_;

    // Services
    ros::ServiceServer srv_shutdown_;

    // Timers
    ros::Timer battery_timer_;

    // Camera
    std::unique_ptr<X5Camera> camera_;

    // Parameters
    std::string preview_resolution_;
    std::string recording_resolution_;
    bool auto_start_recording_;
    int battery_poll_interval_sec_;
    int low_battery_threshold_percent_;
    std::string lens0_frame_id_;
    std::string lens1_frame_id_;
    std::string imu_frame_id_;
    bool publish_combined_;
    std::string combined_format_;

    // State
    std::atomic<bool> running_{false};
    std::atomic<bool> low_battery_triggered_{false};

    // Stats
    uint64_t imu_count_ = 0;
    uint64_t lens0_count_ = 0;
    uint64_t lens1_count_ = 0;

    // Frame synchronization for combined image
    cv::Mat last_lens0_frame_;
    cv::Mat last_lens1_frame_;
    double last_lens0_timestamp_ = 0;
    double last_lens1_timestamp_ = 0;
    std::mutex frame_sync_mutex_;
};

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ros::init(argc, argv, "x5_driver_node", ros::init_options::NoSigintHandler);
    
    // Setup signal handlers for clean shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ROS_INFO("========================================");
    ROS_INFO("   Insta360 X5 ROS Driver");
    ROS_INFO("========================================");

    X5DriverNode node(nh, pnh);

    if (!node.start()) {
        ROS_ERROR("Failed to start X5 driver node");
        return 1;
    }

    node.spin();
    node.shutdown();

    ROS_INFO("X5 driver node exited cleanly");
    return 0;
}
