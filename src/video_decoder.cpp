#include "x5_ros_driver/video_decoder.hpp"
#include <iostream>

namespace x5_ros_driver {

VideoDecoder::VideoDecoder() {
    // Nothing to do - init() must be called
}

VideoDecoder::~VideoDecoder() {
    cleanup();
}

void VideoDecoder::cleanup() {
    std::lock_guard<std::mutex> lock(mutex_);

    // Release OpenCV buffers
    yuv_buffer_.release();
    bgr_buffer_mat_.release();

    if (frame_) {
        av_frame_free(&frame_);
        frame_ = nullptr;
    }

    if (packet_) {
        av_packet_free(&packet_);
        packet_ = nullptr;
    }

    if (codec_ctx_) {
        avcodec_free_context(&codec_ctx_);
        codec_ctx_ = nullptr;
    }

    initialized_ = false;
    width_ = 0;
    height_ = 0;
}

bool VideoDecoder::init(VideoCodecType type) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_) {
        cleanup();
    }

    // Suppress verbose FFmpeg warnings (optional - removes the mmco spam from logs)
    av_log_set_level(AV_LOG_FATAL);

    codec_type_ = type;

    // Find decoder
    AVCodecID codec_id = (type == VideoCodecType::H264) ? AV_CODEC_ID_H264 : AV_CODEC_ID_HEVC;
    codec_ = avcodec_find_decoder(codec_id);
    if (!codec_) {
        std::cerr << "[VideoDecoder] Could not find "
                  << (type == VideoCodecType::H264 ? "H.264" : "H.265")
                  << " decoder" << std::endl;
        return false;
    }

    // Allocate codec context
    codec_ctx_ = avcodec_alloc_context3(codec_);
    if (!codec_ctx_) {
        std::cerr << "[VideoDecoder] Could not allocate codec context" << std::endl;
        return false;
    }

    // Configure decoder BEFORE opening
    codec_ctx_->flags |= AV_CODEC_FLAG_LOW_DELAY;
    codec_ctx_->flags2 |= AV_CODEC_FLAG2_FAST;
    codec_ctx_->refs = 16;  // Allow up to 16 reference frames
    codec_ctx_->err_recognition = 0;  // Be tolerant of errors

    // Enable multi-threaded decoding for better 4K performance
    codec_ctx_->thread_count = 0;  // 0 = auto-detect optimal thread count
    codec_ctx_->thread_type = FF_THREAD_FRAME | FF_THREAD_SLICE;  // Use both frame and slice threading

    // Open codec
    if (avcodec_open2(codec_ctx_, codec_, nullptr) < 0) {
        std::cerr << "[VideoDecoder] Could not open codec" << std::endl;
        cleanup();
        return false;
    }

    // Allocate frame for decoded YUV
    frame_ = av_frame_alloc();
    if (!frame_) {
        std::cerr << "[VideoDecoder] Could not allocate frame" << std::endl;
        cleanup();
        return false;
    }

    // Allocate packet
    packet_ = av_packet_alloc();
    if (!packet_) {
        std::cerr << "[VideoDecoder] Could not allocate packet" << std::endl;
        cleanup();
        return false;
    }

    initialized_ = true;
    std::cout << "[VideoDecoder] Initialized " 
              << (type == VideoCodecType::H264 ? "H.264" : "H.265") 
              << " decoder" << std::endl;

    return true;
}

bool VideoDecoder::initSwsContext(int width, int height) {
    // Check if dimensions changed
    if (width == width_ && height == height_ && !yuv_buffer_.empty()) {
        return true;  // Already initialized
    }

    width_ = width;
    height_ = height;

    // Pre-allocate YUV buffer for OpenCV conversion (I420 format: height * 1.5)
    yuv_buffer_.create(height + height / 2, width, CV_8UC1);

    // Pre-allocate BGR output buffer (avoids allocation in cvtColor)
    bgr_buffer_mat_.create(height, width, CV_8UC3);

    std::cout << "[VideoDecoder] Initialized OpenCV buffers for "
              << width << "x" << height << " (using SIMD-optimized cvtColor)" << std::endl;

    return true;
}

bool VideoDecoder::decode(const uint8_t* data, size_t size, cv::Mat& output) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        std::cerr << "[VideoDecoder] Not initialized" << std::endl;
        return false;
    }

    // Setup packet with input data
    packet_->data = const_cast<uint8_t*>(data);
    packet_->size = static_cast<int>(size);

    // Send packet to decoder
    int ret = avcodec_send_packet(codec_ctx_, packet_);
    if (ret < 0) {
        // Don't spam errors - some packets are expected to fail
        // (e.g., first packets before SPS/PPS)
        return false;
    }

    // Try to receive decoded frame
    ret = avcodec_receive_frame(codec_ctx_, frame_);
    if (ret < 0) {
        if (ret == AVERROR(EAGAIN)) {
            // Need more data
            return false;
        }
        // Actual error
        return false;
    }

    // Initialize buffers if needed (now we know dimensions)
    if (!initSwsContext(frame_->width, frame_->height)) {
        return false;
    }

    // Copy YUV planes to contiguous buffer for OpenCV (I420 format)
    // This is faster than sws_scale because OpenCV's cvtColor uses SIMD
    uint8_t* yuv_ptr = yuv_buffer_.data;
    int y_size = width_ * height_;
    int uv_width = width_ / 2;
    int uv_height = height_ / 2;

    // Copy Y plane (handle stride if different from width)
    if (frame_->linesize[0] == width_) {
        memcpy(yuv_ptr, frame_->data[0], y_size);
    } else {
        for (int i = 0; i < height_; i++) {
            memcpy(yuv_ptr + i * width_, frame_->data[0] + i * frame_->linesize[0], width_);
        }
    }
    yuv_ptr += y_size;

    // Copy U plane
    if (frame_->linesize[1] == uv_width) {
        memcpy(yuv_ptr, frame_->data[1], uv_width * uv_height);
    } else {
        for (int i = 0; i < uv_height; i++) {
            memcpy(yuv_ptr + i * uv_width, frame_->data[1] + i * frame_->linesize[1], uv_width);
        }
    }
    yuv_ptr += uv_width * uv_height;

    // Copy V plane
    if (frame_->linesize[2] == uv_width) {
        memcpy(yuv_ptr, frame_->data[2], uv_width * uv_height);
    } else {
        for (int i = 0; i < uv_height; i++) {
            memcpy(yuv_ptr + i * uv_width, frame_->data[2] + i * frame_->linesize[2], uv_width);
        }
    }

    // Convert YUV I420 to BGR using OpenCV's SIMD-optimized cvtColor
    cv::cvtColor(yuv_buffer_, bgr_buffer_mat_, cv::COLOR_YUV2BGR_I420);

    // Clone output (caller may hold reference across decode calls)
    output = bgr_buffer_mat_.clone();

    return true;
}

} // namespace x5_ros_driver
