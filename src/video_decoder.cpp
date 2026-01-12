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

    if (sws_ctx_) {
        sws_freeContext(sws_ctx_);
        sws_ctx_ = nullptr;
    }

    if (bgr_buffer_) {
        av_free(bgr_buffer_);
        bgr_buffer_ = nullptr;
    }

    if (frame_bgr_) {
        av_frame_free(&frame_bgr_);
        frame_bgr_ = nullptr;
    }

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
}

bool VideoDecoder::init(VideoCodecType type) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_) {
        cleanup();
    }

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

    // Set some options for low-latency decoding
    codec_ctx_->flags |= AV_CODEC_FLAG_LOW_DELAY;
    codec_ctx_->flags2 |= AV_CODEC_FLAG2_FAST;

    // Open codec
    if (avcodec_open2(codec_ctx_, codec_, nullptr) < 0) {
        std::cerr << "[VideoDecoder] Could not open codec" << std::endl;
        cleanup();
        return false;
    }

    // Allocate frames
    frame_ = av_frame_alloc();
    frame_bgr_ = av_frame_alloc();
    if (!frame_ || !frame_bgr_) {
        std::cerr << "[VideoDecoder] Could not allocate frames" << std::endl;
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
    // Free existing context if dimensions changed
    if (sws_ctx_ && (width != width_ || height != height_)) {
        sws_freeContext(sws_ctx_);
        sws_ctx_ = nullptr;
        if (bgr_buffer_) {
            av_free(bgr_buffer_);
            bgr_buffer_ = nullptr;
        }
    }

    if (sws_ctx_) {
        return true;  // Already initialized with correct dimensions
    }

    width_ = width;
    height_ = height;

    // Create scaling context for YUV -> BGR conversion
    sws_ctx_ = sws_getContext(
        width, height, codec_ctx_->pix_fmt,
        width, height, AV_PIX_FMT_BGR24,
        SWS_BILINEAR, nullptr, nullptr, nullptr
    );

    if (!sws_ctx_) {
        std::cerr << "[VideoDecoder] Could not create SwsContext" << std::endl;
        return false;
    }

    // Allocate BGR buffer
    int bgr_size = av_image_get_buffer_size(AV_PIX_FMT_BGR24, width, height, 1);
    bgr_buffer_ = (uint8_t*)av_malloc(bgr_size);
    if (!bgr_buffer_) {
        std::cerr << "[VideoDecoder] Could not allocate BGR buffer" << std::endl;
        return false;
    }

    // Setup frame_bgr_ with the buffer
    av_image_fill_arrays(
        frame_bgr_->data, frame_bgr_->linesize,
        bgr_buffer_, AV_PIX_FMT_BGR24,
        width, height, 1
    );

    std::cout << "[VideoDecoder] Initialized SwsContext for " 
              << width << "x" << height << std::endl;

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

    // Initialize SwsContext if needed (now we know dimensions)
    if (!initSwsContext(frame_->width, frame_->height)) {
        return false;
    }

    // Convert YUV to BGR
    sws_scale(
        sws_ctx_,
        frame_->data, frame_->linesize,
        0, frame_->height,
        frame_bgr_->data, frame_bgr_->linesize
    );

    // Copy to OpenCV Mat
    output = cv::Mat(height_, width_, CV_8UC3, frame_bgr_->data[0], frame_bgr_->linesize[0]).clone();

    return true;
}

} // namespace x5_ros_driver
