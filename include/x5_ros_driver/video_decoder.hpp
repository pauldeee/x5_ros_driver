#pragma once

#include <opencv2/opencv.hpp>
#include <memory>
#include <mutex>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

namespace x5_ros_driver {

enum class VideoCodecType {
    H264,
    H265
};

/**
 * @brief FFmpeg-based video decoder for H.264/H.265 streams
 * 
 * Decodes raw NAL units from the Insta360 SDK into OpenCV Mat images.
 */
class VideoDecoder {
public:
    VideoDecoder();
    ~VideoDecoder();

    // Disable copy
    VideoDecoder(const VideoDecoder&) = delete;
    VideoDecoder& operator=(const VideoDecoder&) = delete;

    /**
     * @brief Initialize the decoder for a specific codec
     * @param type H264 or H265
     * @return true if initialization successful
     */
    bool init(VideoCodecType type);

    /**
     * @brief Decode a video packet
     * @param data Pointer to encoded data (NAL units)
     * @param size Size of encoded data
     * @param output Output OpenCV Mat (BGR format)
     * @return true if a frame was decoded, false otherwise
     */
    bool decode(const uint8_t* data, size_t size, cv::Mat& output);

    /**
     * @brief Check if decoder is initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Get the decoded frame dimensions
     */
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }

private:
    bool initialized_ = false;
    VideoCodecType codec_type_;

    // FFmpeg contexts
    const AVCodec* codec_ = nullptr;
    AVCodecContext* codec_ctx_ = nullptr;
    AVFrame* frame_ = nullptr;
    AVFrame* frame_bgr_ = nullptr;
    AVPacket* packet_ = nullptr;
    SwsContext* sws_ctx_ = nullptr;

    // Frame dimensions (set after first decode)
    int width_ = 0;
    int height_ = 0;

    // Buffer for BGR conversion
    uint8_t* bgr_buffer_ = nullptr;

    std::mutex mutex_;

    void cleanup();
    bool initSwsContext(int width, int height);
};

} // namespace x5_ros_driver
