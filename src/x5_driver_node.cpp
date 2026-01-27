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
#include <deque>
#include <thread>
#include <chrono>
#include <execinfo.h>
#include <cxxabi.h>
#include <cstdlib>

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
    X5DriverNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : nh_(nh), pnh_(pnh), it_(nh) {
        // Load parameters
        loadParameters();

        // Create publishers
        pub_lens0_ = it_.advertise("x5/lens0", 1);
        pub_lens1_ = it_.advertise("x5/lens1", 1);
        pub_combined_ = it_.advertise("x5/combined", 1);
        pub_imu_ = nh_.advertise<sensor_msgs::Imu>("x5/imu", 100);
        pub_exposure_ = nh_.advertise<sensor_msgs::TimeReference>("x5/exposure", 10);
        pub_recording_info_ = nh_.advertise<std_msgs::String>("x5/recording_info", 1, true); // Latched
        pub_battery_ = nh_.advertise<std_msgs::Int32>("x5/battery_level", 1, true); // Latched

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
        camera_ = nullptr; // Use g_camera from now on

        // Connect to camera
        if (!g_camera->connect()) {
            ROS_ERROR("Failed to connect to X5 camera");
            return false;
        }

        // Sync camera time to system time
        if (sync_time_on_connect_) {
            if (g_camera->syncTimeToSystem(100)) {
                // 100ms tolerance
                ROS_INFO("Camera time synchronized to system time");
            } else {
                ROS_WARN("Camera time sync failed or offset too large - timestamps may be inaccurate");
            }
        }

        // Set callbacks
        g_camera->setImuCallback([this](const ImuSample &imu) {
            publishImu(imu);
        });

        g_camera->setVideoCallback([this](const VideoFrame &frame) {
            publishVideo(frame);
        });

        g_camera->setExposureCallback([this](const ExposureInfo &exp) {
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

        // Subscribe to GPS time if enabled
        if (use_gps_time_) {
            sub_gps_time_ = nh_.subscribe(gps_time_topic_, 10, &X5DriverNode::gpsTimeCallback, this);
            ROS_INFO("Subscribing to GPS time: %s", gps_time_topic_.c_str());
            ROS_INFO("Waiting for first GPS time message...");
        }

        // Subscribe to camera trigger if enabled
        if (use_trigger_sync_) {
            sub_camera_trigger_ = nh_.subscribe(camera_trigger_topic_, 100,
                                                &X5DriverNode::cameraTriggerCallback, this);
            ROS_INFO("Subscribing to camera trigger: %s", camera_trigger_topic_.c_str());
            ROS_INFO("Images will only be published when synchronized with triggers (~10 Hz)");

            // Set frame filter to skip cloning frames that won't match any trigger.
            // The H.264 decode still runs for every frame (required for reference chain),
            // but the expensive clone (~22MB at 4K) is skipped for ~20/30 frames per second.
            g_camera->setFrameFilter([this](int64_t timestamp_ms) -> bool {
                // Convert camera time to system time (atomic read, no mutex needed)
                int64_t sys_ms = g_camera->cameraTimeToSystemTime(timestamp_ms);

                std::lock_guard<std::mutex> lock(trigger_mutex_);

                if (waiting_for_first_trigger_) {
                    return false;  // No triggers yet, skip clone
                }

                // Check if there's an unconsumed trigger at-or-before this frame
                for (const auto &trigger : trigger_buffer_) {
                    if (trigger.consumed) continue;

                    int64_t trigger_ms = static_cast<int64_t>(trigger.ros_stamp.sec) * 1000
                                         + trigger.ros_stamp.nsec / 1000000;
                    if (sys_ms < trigger_ms) {
                        return false;  // Frame is before earliest unconsumed trigger
                    }

                    // Frame is at-or-after this trigger — clone it for matching
                    return true;
                }

                // No unconsumed triggers remain — skip
                return false;
            });
        }

        // Get battery
        if (enable_battery_monitoring_) {
            int battery = g_camera->getBatteryLevel();
            if (battery >= 0) {
                ROS_INFO("Battery: %d%%", battery);
                if (battery < 20) {
                    ROS_WARN("Low battery! %d%%", battery);
                }
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

        if (!g_camera->startStreaming(preview_res, auto_start_recording_, rec_res, use_hardware_accel_)) {
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

        // Start background thread for periodic SDK tasks (battery monitoring).
        // The camera→system time offset from initial sync is stable (±3ms drift
        // observed over minutes), so we don't refresh it during streaming.
        // Refreshing it requires GetCameraMediaTime() — a synchronous SDK/USB call
        // that can conflict with the streaming data path and crash the SDK.
        if (enable_battery_monitoring_) {
            startTimeRefreshThread();
        }

        return true;
    }

    void spin() {
        ros::Rate rate(100); // 100 Hz check rate

        while (ros::ok() && !g_shutdown_requested && running_) {
            ros::spinOnce();
            rate.sleep();

            // Periodic connection check
            static int counter = 0;
            if (++counter >= 1000) {
                // Every ~10 seconds
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
        stopTimeRefreshThread();
        ROS_INFO("Shutting down X5 driver...");

        if (g_camera) {
            auto recorded_files = g_camera->stopStreaming();

            // Publish recording info
            if (!recorded_files.empty()) {
                std_msgs::String msg;
                msg.data = "Recorded files:\n";
                for (const auto &file: recorded_files) {
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
        pnh_.param<bool>("enable_battery_monitoring", enable_battery_monitoring_, true);
        pnh_.param<int>("battery_poll_interval_sec", battery_poll_interval_sec_, 30);
        pnh_.param<int>("low_battery_threshold_percent", low_battery_threshold_percent_, 15);
        pnh_.param<std::string>("lens0_frame_id", lens0_frame_id_, "x5_lens0");
        pnh_.param<std::string>("lens1_frame_id", lens1_frame_id_, "x5_lens1");
        pnh_.param<std::string>("imu_frame_id", imu_frame_id_, "x5_imu");
        pnh_.param<bool>("publish_combined", publish_combined_, true);
        pnh_.param<std::string>("combined_format", combined_format_, "sidebyside");
        pnh_.param<bool>("sync_time_on_connect", sync_time_on_connect_, true);
        pnh_.param<bool>("use_gps_time", use_gps_time_, false);
        pnh_.param<std::string>("gps_time_topic", gps_time_topic_, "/ext/time");
        pnh_.param<bool>("use_trigger_sync", use_trigger_sync_, false);
        pnh_.param<std::string>("camera_trigger_topic", camera_trigger_topic_, "/camera_trigger_time");
        pnh_.param<double>("trigger_max_age_sec", trigger_max_age_sec_, 0.20);
        pnh_.param<int>("trigger_gps_match_threshold_ms", trigger_gps_match_threshold_ms_, 70);
        pnh_.param<bool>("use_hardware_accel", use_hardware_accel_, true);

        ROS_INFO("Parameters:");
        ROS_INFO("  preview_resolution: %s", preview_resolution_.c_str());
        ROS_INFO("  recording_resolution: %s", recording_resolution_.c_str());
        ROS_INFO("  auto_start_recording: %s", auto_start_recording_ ? "true" : "false");
        ROS_INFO("  enable_battery_monitoring: %s", enable_battery_monitoring_ ? "true" : "false");
        if (enable_battery_monitoring_) {
            ROS_INFO("  battery_poll_interval_sec: %d", battery_poll_interval_sec_);
            ROS_INFO("  low_battery_threshold_percent: %d%%", low_battery_threshold_percent_);
        }
        ROS_INFO("  use_hardware_accel: %s", use_hardware_accel_ ? "true" : "false");
        ROS_INFO("  sync_time_on_connect: %s", sync_time_on_connect_ ? "true" : "false");
        ROS_INFO("  use_gps_time: %s", use_gps_time_ ? "true" : "false");
        if (use_gps_time_) {
            ROS_INFO("  gps_time_topic: %s", gps_time_topic_.c_str());
        }
        ROS_INFO("  use_trigger_sync: %s", use_trigger_sync_ ? "true" : "false");
        if (use_trigger_sync_) {
            ROS_INFO("  camera_trigger_topic: %s", camera_trigger_topic_.c_str());
            ROS_INFO("  trigger_max_age_sec: %.3f", trigger_max_age_sec_);
            ROS_INFO("  trigger_gps_match_threshold_ms: %d", trigger_gps_match_threshold_ms_);
        }
    }

    void publishImu(const ImuSample &imu) {
        sensor_msgs::Imu msg;

        // Convert camera time to GPS time (or system time if GPS not available)
        int64_t output_time_ms = g_camera->cameraTimeToGpsTime(static_cast<int64_t>(imu.timestamp_ms));
        msg.header.stamp.sec = output_time_ms / 1000;
        msg.header.stamp.nsec = (output_time_ms % 1000) * 1000000;
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
        msg.orientation_covariance[0] = -1; // Orientation not provided
        msg.angular_velocity_covariance[0] = 0;
        msg.linear_acceleration_covariance[0] = 0;

        pub_imu_.publish(msg);
        imu_count_++;
    }

    void publishVideo(const VideoFrame &frame) {
        if (frame.image.empty()) {
            return;
        }

        // Convert camera time to system/ROS time for trigger matching.
        // This uses only the camera→system offset (single, stable offset)
        // rather than camera→system→GPS (two offsets, prone to drift).
        int64_t system_time_ms = g_camera->cameraTimeToSystemTime(static_cast<int64_t>(frame.timestamp_ms));
        ros::Time frame_sys_time;
        frame_sys_time.sec = system_time_ms / 1000;
        frame_sys_time.nsec = (system_time_ms % 1000) * 1000000;

        ros::Time stamp = frame_sys_time; // Default: use system time
        bool should_publish = true;

        // Trigger synchronization logic: forced 1:1 matching.
        // Every trigger MUST produce one image (the first frame at-or-after the trigger).
        // No threshold, no stale triggers — at 30fps there's always a frame within 33ms.
        if (use_trigger_sync_) {
            // Compute GPS time OUTSIDE trigger_mutex_ to avoid nested lock contention
            int64_t frame_gps_ms = 0;
            bool have_gps = g_camera && g_camera->isGpsTimeOffsetValid();
            if (have_gps) {
                frame_gps_ms = g_camera->cameraTimeToGpsTime(static_cast<int64_t>(frame.timestamp_ms));
            }

            std::lock_guard<std::mutex> lock(trigger_mutex_);

            // Before first trigger: discard all frames
            if (waiting_for_first_trigger_) {
                frames_discarded_++;
                return;
            }

            // Find oldest unconsumed trigger where this frame is at-or-after trigger time.
            // Match unconditionally — forced 1:1 pairing.
            TriggerInfo *matched_trigger = nullptr;

            for (size_t i = 0; i < trigger_buffer_.size(); i++) {
                TriggerInfo &trigger = trigger_buffer_[i];
                if (trigger.consumed) continue;

                if (have_gps) {
                    // GPS-domain matching
                    int64_t diff_ms = frame_gps_ms - trigger.gps_stamp_ms;
                    if (diff_ms < 0) break;  // Frame is before this trigger — stop
                    matched_trigger = &trigger;
                } else {
                    // System-time fallback
                    double diff_sec = (frame_sys_time - trigger.ros_stamp).toSec();
                    if (diff_sec < 0) break;
                    matched_trigger = &trigger;
                }
                break;  // Take the first eligible trigger
            }

            if (matched_trigger) {
                // Stamp with frame's GPS time (camera capture time in GPS domain)
                if (have_gps) {
                    stamp.sec = frame_gps_ms / 1000;
                    stamp.nsec = (frame_gps_ms % 1000) * 1000000;
                } else {
                    // Fallback: use trigger GPS time + capture offset
                    ros::Duration capture_offset = frame_sys_time - matched_trigger->ros_stamp;
                    stamp = matched_trigger->gps_stamp + capture_offset;
                }

                if (frames_matched_ == 0) {
                    const char *mode = have_gps ? "[GPS]" : "[SYS]";
                    ROS_INFO("FIRST trigger match %s: "
                             "frame_sys=%.3f trigger_ros=%.3f trigger_gps=%.3f | "
                             "cam_offset=%ld gps_offset=%s",
                             mode,
                             frame_sys_time.toSec(), matched_trigger->ros_stamp.toSec(),
                             matched_trigger->gps_stamp.toSec(),
                             g_camera ? g_camera->getTimeOffset() : 0,
                             have_gps ? std::to_string(g_camera->getGpsTimeOffset()).c_str() : "N/A");
                }

                // Mark trigger consumed only after lens1 (stream_index == 1)
                // Both lenses arrive with same timestamp, so lens0 comes first
                if (frame.stream_index == 1) {
                    matched_trigger->consumed = true;
                    frames_matched_++;

                    // Clean up consumed triggers from front of buffer
                    while (!trigger_buffer_.empty() && trigger_buffer_.front().consumed) {
                        trigger_buffer_.pop_front();
                    }
                }
            } else {
                // No matching trigger - discard frame
                frames_discarded_++;
                should_publish = false;
            }
        }

        if (!should_publish) {
            return;
        }

        // Create image message
        std_msgs::Header header;
        header.stamp = stamp;
        header.frame_id = (frame.stream_index == 0) ? lens0_frame_id_ : lens1_frame_id_;

        cv_bridge::CvImage cv_img(header, "bgr8", frame.image);
        sensor_msgs::ImagePtr img_msg = cv_img.toImageMsg();

        // Publish to appropriate topic and copy directly into pre-allocated combined buffer
        if (frame.stream_index == 0) {
            pub_lens0_.publish(img_msg);
            lens0_count_++;

            if (publish_combined_) {
                std::lock_guard<std::mutex> lock(frame_sync_mutex_);
                ensureCombinedBufferInitialized(frame.image.size());
                // Copy directly into left/top region of combined buffer (avoids intermediate clone)
                if (combined_format_ == "topbottom") {
                    frame.image.copyTo(combined_buffer_(cv::Rect(0, 0, frame.image.cols, frame.image.rows)));
                } else {
                    frame.image.copyTo(combined_buffer_(cv::Rect(0, 0, frame.image.cols, frame.image.rows)));
                }
                last_lens0_timestamp_ = frame.timestamp_ms;
                last_lens0_stamp_ = stamp;
            }
        } else {
            pub_lens1_.publish(img_msg);
            lens1_count_++;

            if (publish_combined_) {
                std::lock_guard<std::mutex> lock(frame_sync_mutex_);
                ensureCombinedBufferInitialized(frame.image.size());
                // Copy directly into right/bottom region of combined buffer (avoids intermediate clone)
                if (combined_format_ == "topbottom") {
                    frame.image.copyTo(
                        combined_buffer_(cv::Rect(0, frame.image.rows, frame.image.cols, frame.image.rows)));
                } else {
                    frame.image.copyTo(
                        combined_buffer_(cv::Rect(frame.image.cols, 0, frame.image.cols, frame.image.rows)));
                }
                last_lens1_timestamp_ = frame.timestamp_ms;
                last_lens1_stamp_ = stamp;
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
            if (use_trigger_sync_) {
                const char *mode = (g_camera && g_camera->isGpsTimeOffsetValid()) ? "[GPS]" : "[SYS]";
                uint64_t pkt_drop = g_camera ? g_camera->getPacketsDropped() : 0;
                uint64_t frm_drop = g_camera ? g_camera->getFramesDropped() : 0;
                ROS_INFO("Sync stats %s - triggers: %lu, matched: %lu, discarded_frames: %lu | "
                         "lens0: %lu, lens1: %lu, imu: %lu | "
                         "pkt_drop: %lu, frm_drop: %lu | "
                         "cam_offset: %ld ms | gps_offset: %s",
                         mode,
                         triggers_received_, frames_matched_, frames_discarded_,
                         lens0_count_, lens1_count_, imu_count_,
                         pkt_drop, frm_drop,
                         g_camera ? g_camera->getTimeOffset() : 0,
                         (g_camera && g_camera->isGpsTimeOffsetValid())
                             ? std::to_string(g_camera->getGpsTimeOffset()).c_str()
                             : "N/A");
            } else {
                ROS_INFO("Published - lens0: %lu, lens1: %lu, imu: %lu",
                         lens0_count_, lens1_count_, imu_count_);
            }
        }
    }

    void tryPublishCombined() {
        std::lock_guard<std::mutex> lock(frame_sync_mutex_);

        // Need both frames (timestamp < 0 means not yet received this cycle)
        if (last_lens0_timestamp_ < 0 || last_lens1_timestamp_ < 0) {
            return;
        }

        // Check timestamps match (within 1ms - should be same frame)
        if (std::abs(last_lens0_timestamp_ - last_lens1_timestamp_) > 1.0) {
            return;
        }

        // Publish pre-built combined buffer directly (no hconcat/vconcat allocation!)
        std_msgs::Header header;
        header.stamp = last_lens0_stamp_;
        header.frame_id = "x5_combined";

        cv_bridge::CvImage cv_img(header, "bgr8", combined_buffer_);
        pub_combined_.publish(cv_img.toImageMsg());

        // Reset timestamps to indicate frames consumed
        last_lens0_timestamp_ = -1;
        last_lens1_timestamp_ = -1;
    }

    void publishExposure(const ExposureInfo &exp) {
        sensor_msgs::TimeReference msg;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = lens0_frame_id_;

        // Store converted X5 timestamp in time_ref
        int64_t output_time_ms = g_camera->cameraTimeToGpsTime(static_cast<int64_t>(exp.timestamp_ms));
        msg.time_ref.sec = output_time_ms / 1000;
        msg.time_ref.nsec = (output_time_ms % 1000) * 1000000;

        // Store exposure time in source field
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

    bool shutdownCallback(std_srvs::Trigger::Request &req,
                          std_srvs::Trigger::Response &res) {
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
                for (const auto &file: recorded_files) {
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

    void gpsTimeCallback(const sensor_msgs::TimeReference::ConstPtr &msg) {
        if (!g_camera) return;

        int64_t system_time_ms = static_cast<int64_t>(msg->header.stamp.sec) * 1000
                                 + msg->header.stamp.nsec / 1000000;
        int64_t gps_time_ms = static_cast<int64_t>(msg->time_ref.sec) * 1000
                              + msg->time_ref.nsec / 1000000;

        g_camera->setGpsTimeOffset(system_time_ms, gps_time_ms);
        // Note: refreshTimeOffset() runs on a background thread to avoid
        // blocking spinOnce() with a synchronous SDK USB call, which would
        // delay trigger callback delivery and cause stale triggers.
    }

    void cameraTriggerCallback(const sensor_msgs::TimeReference::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(trigger_mutex_);

        TriggerInfo trigger;
        trigger.ros_stamp = msg->header.stamp;
        trigger.gps_stamp = msg->time_ref;
        trigger.gps_stamp_ms = static_cast<int64_t>(msg->time_ref.sec) * 1000
                               + msg->time_ref.nsec / 1000000;
        trigger.consumed = false;

        trigger_buffer_.push_back(trigger);
        triggers_received_++;

        // Trim buffer (keep last 100)
        while (trigger_buffer_.size() > 100) {
            trigger_buffer_.pop_front();
        }

        if (waiting_for_first_trigger_) {
            waiting_for_first_trigger_ = false;
            ROS_INFO("First camera trigger received - starting synchronized publishing");
        }
    }

    void startTimeRefreshThread() {
        time_refresh_running_ = true;
        time_refresh_thread_ = std::thread([this]() {
            while (time_refresh_running_) {
                try {
                    if (g_camera && g_camera->isConnected()) {
                        publishBatteryLevel();
                    }
                } catch (const std::exception& e) {
                    ROS_WARN_THROTTLE(10, "Background SDK call failed: %s", e.what());
                } catch (...) {
                    ROS_WARN_THROTTLE(10, "Background SDK call failed (unknown exception)");
                }
                // Sleep for battery_poll_interval, checking for shutdown every 50ms
                int sleep_iters = battery_poll_interval_sec_ * 20;  // 50ms per iter
                for (int i = 0; i < sleep_iters && time_refresh_running_; i++) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }
        });
    }

    void stopTimeRefreshThread() {
        time_refresh_running_ = false;
        if (time_refresh_thread_.joinable()) {
            time_refresh_thread_.join();
        }
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

    // Camera
    std::unique_ptr<X5Camera> camera_;

    // Parameters
    std::string preview_resolution_;
    std::string recording_resolution_;
    bool auto_start_recording_;
    bool enable_battery_monitoring_;
    int battery_poll_interval_sec_;
    int low_battery_threshold_percent_;
    std::string lens0_frame_id_;
    std::string lens1_frame_id_;
    std::string imu_frame_id_;
    bool publish_combined_;
    std::string combined_format_;
    bool sync_time_on_connect_;
    bool use_hardware_accel_;

    // GPS time sync
    bool use_gps_time_;
    std::string gps_time_topic_;
    ros::Subscriber sub_gps_time_;

    // Trigger synchronization
    bool use_trigger_sync_;
    std::string camera_trigger_topic_;
    double trigger_max_age_sec_;
    int trigger_gps_match_threshold_ms_ = 95;
    ros::Subscriber sub_camera_trigger_;

    // Trigger state (protected by trigger_mutex_)
    struct TriggerInfo {
        ros::Time ros_stamp; // header.stamp - ROS time when trigger arrived
        ros::Time gps_stamp; // time_ref - GPS time to stamp the image
        int64_t gps_stamp_ms; // time_ref in milliseconds for fast arithmetic
        bool consumed; // Whether this trigger has been used
    };

    std::deque<TriggerInfo> trigger_buffer_;
    std::mutex trigger_mutex_;
    bool waiting_for_first_trigger_ = true;

    // Trigger sync stats
    uint64_t triggers_received_ = 0;
    uint64_t frames_matched_ = 0;
    uint64_t frames_discarded_ = 0;

    // State
    std::atomic<bool> running_{false};
    std::atomic<bool> low_battery_triggered_{false};
    std::atomic<bool> time_refresh_running_{false};
    std::thread time_refresh_thread_;

    // Stats
    uint64_t imu_count_ = 0;
    uint64_t lens0_count_ = 0;
    uint64_t lens1_count_ = 0;

    // Frame synchronization for combined image (optimized zero-copy path)
    cv::Mat combined_buffer_; // Pre-allocated buffer for combined image
    bool combined_buffer_initialized_ = false;
    double last_lens0_timestamp_ = -1; // -1 = not set
    double last_lens1_timestamp_ = -1; // -1 = not set
    ros::Time last_lens0_stamp_;
    ros::Time last_lens1_stamp_;
    std::mutex frame_sync_mutex_;

    // Helper to lazily initialize combined buffer
    void ensureCombinedBufferInitialized(const cv::Size &lens_size) {
        if (combined_buffer_initialized_ &&
            ((combined_format_ == "topbottom" && combined_buffer_.rows == lens_size.height * 2) ||
             (combined_format_ != "topbottom" && combined_buffer_.cols == lens_size.width * 2))) {
            return;
        }

        if (combined_format_ == "topbottom") {
            combined_buffer_ = cv::Mat(lens_size.height * 2, lens_size.width, CV_8UC3);
        } else {
            combined_buffer_ = cv::Mat(lens_size.height, lens_size.width * 2, CV_8UC3);
        }
        combined_buffer_initialized_ = true;
        ROS_INFO("Initialized combined buffer: %dx%d", combined_buffer_.cols, combined_buffer_.rows);
    }
};

// ============================================================================
// Main
// ============================================================================

void terminateHandler() {
    std::cerr << "\n=== TERMINATE HANDLER ===" << std::endl;

    // Print current exception info
    if (auto eptr = std::current_exception()) {
        try {
            std::rethrow_exception(eptr);
        } catch (const std::exception& e) {
            std::cerr << "Exception: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "Unknown exception" << std::endl;
        }
    }

    // Print stack trace
    void* frames[64];
    int count = backtrace(frames, 64);
    char** symbols = backtrace_symbols(frames, count);
    std::cerr << "Stack trace (" << count << " frames):" << std::endl;
    for (int i = 0; i < count; i++) {
        std::cerr << "  [" << i << "] " << symbols[i] << std::endl;
    }
    free(symbols);

    std::cerr << "=== END TERMINATE HANDLER ===\n" << std::endl;
    std::abort();
}

int main(int argc, char **argv) {
    std::set_terminate(terminateHandler);
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
