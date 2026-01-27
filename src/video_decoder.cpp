#include "x5_ros_driver/video_decoder.hpp"
#include <iostream>

// Static pointer for get_format callback (VAAPI requires this pattern)
static enum AVPixelFormat s_hw_pix_fmt = AV_PIX_FMT_NONE;

namespace x5_ros_driver {

VideoDecoder::VideoDecoder() {
    // Nothing to do - init() must be called
}



    // FIS
VideoDecoder::~VideoDecoder() {
    cleanup();
}

void VideoDecoder::cleanup() {
    std::lock_guard<std::mutex> lock(mutex_);

    // Release OpenCV buffers
    yuv_buffer_.release();
    nv12_buffer_.release();
    bgr_buffers_[0].release();
    bgr_buffers_[1].release();

    if (sw_frame_) {
        av_frame_free(&sw_frame_);
        sw_frame_ = nullptr;
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

    if (hw_device_ctx_) {
        av_buffer_unref(&hw_device_ctx_);
        hw_device_ctx_ = nullptr;
    }

    hw_accel_enabled_ = false;
    hw_pix_fmt_ = AV_PIX_FMT_NONE;
    initialized_ = false;
    width_ = 0;
    height_ = 0;
    current_buffer_ = 0;
}

enum AVPixelFormat VideoDecoder::getHwFormat(AVCodecContext* ctx, const enum AVPixelFormat* pix_fmts) {
    // Callback to select hardware pixel format
    for (const enum AVPixelFormat* p = pix_fmts; *p != AV_PIX_FMT_NONE; p++) {
        if (*p == s_hw_pix_fmt) {
            return *p;
        }
    }
    std::cerr << "[VideoDecoder] Failed to get HW surface format, falling back to SW" << std::endl;
    return AV_PIX_FMT_NONE;
}

bool VideoDecoder::initHwAccel() {
    // Try to create VAAPI device context
    int ret = av_hwdevice_ctx_create(&hw_device_ctx_,
                                      AV_HWDEVICE_TYPE_VAAPI,
                                      "/dev/dri/renderD128",
                                      nullptr, 0);
    if (ret < 0) {
        char errbuf[256];
        av_strerror(ret, errbuf, sizeof(errbuf));
        std::cerr << "[VideoDecoder] VAAPI init failed: " << errbuf
                  << " - using CPU decode" << std::endl;
        return false;
    }

    hw_pix_fmt_ = AV_PIX_FMT_VAAPI;
    s_hw_pix_fmt = AV_PIX_FMT_VAAPI;  // Set static for callback

    std::cout << "[VideoDecoder] VAAPI hardware acceleration initialized" << std::endl;
    return true;
}

bool VideoDecoder::init(VideoCodecType type, bool use_hw_accel) {
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

    // Try to enable VAAPI hardware acceleration (if requested)
    if (use_hw_accel && initHwAccel()) {
        codec_ctx_->hw_device_ctx = av_buffer_ref(hw_device_ctx_);
        codec_ctx_->get_format = getHwFormat;
        hw_accel_enabled_ = true;
        // Note: threading is handled by VAAPI driver, not needed for HW decode
    } else {
        // Fallback: Enable multi-threaded decoding for CPU
        codec_ctx_->thread_count = 0;  // 0 = auto-detect optimal thread count
        codec_ctx_->thread_type = FF_THREAD_FRAME | FF_THREAD_SLICE;
        hw_accel_enabled_ = false;
    }

    // Open codec
    if (avcodec_open2(codec_ctx_, codec_, nullptr) < 0) {
        std::cerr << "[VideoDecoder] Could not open codec" << std::endl;
        cleanup();
        return false;
    }

    // Allocate frame for decoded YUV (or VAAPI surface)
    frame_ = av_frame_alloc();
    if (!frame_) {
        std::cerr << "[VideoDecoder] Could not allocate frame" << std::endl;
        cleanup();
        return false;
    }

    // Allocate SW frame for HW→SW transfer (if using VAAPI)
    if (hw_accel_enabled_) {
        sw_frame_ = av_frame_alloc();
        if (!sw_frame_) {
            std::cerr << "[VideoDecoder] Could not allocate SW frame" << std::endl;
            cleanup();
            return false;
        }
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
              << " decoder"
              << (hw_accel_enabled_ ? " with VAAPI hardware acceleration" : " (CPU)")
              << std::endl;

    return true;
}

bool VideoDecoder::initSwsContext(int width, int height) {
    // Check if dimensions changed
    if (width == width_ && height == height_ && !bgr_buffers_[0].empty()) {
        return true;  // Already initialized
    }

    width_ = width;
    height_ = height;

    if (hw_accel_enabled_) {
        // VAAPI outputs NV12 format: Y plane + interleaved UV plane
        // NV12 total size = height * 1.5
        nv12_buffer_.create(height + height / 2, width, CV_8UC1);
    } else {
        // CPU decode outputs I420 format: Y plane + U plane + V plane
        yuv_buffer_.create(height + height / 2, width, CV_8UC1);
    }

    // Pre-allocate double-buffered BGR output (eliminates clone overhead)
    bgr_buffers_[0].create(height, width, CV_8UC3);
    bgr_buffers_[1].create(height, width, CV_8UC3);

    std::cout << "[VideoDecoder] Initialized OpenCV buffers for "
              << width << "x" << height
              << (hw_accel_enabled_ ? " (NV12 from VAAPI)" : " (I420 from CPU)")
              << " with double-buffering" << std::endl;

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

    // Pointer to the frame we'll convert (may be HW or SW frame)
    AVFrame* src_frame = frame_;

    // If using hardware acceleration, transfer from GPU to system memory
    if (hw_accel_enabled_ && frame_->format == hw_pix_fmt_) {
        // Release previous sw_frame data before transfer
        av_frame_unref(sw_frame_);

        ret = av_hwframe_transfer_data(sw_frame_, frame_, 0);
        if (ret < 0) {
            char errbuf[256];
            av_strerror(ret, errbuf, sizeof(errbuf));
            std::cerr << "[VideoDecoder] HW frame transfer failed: " << errbuf << std::endl;
            av_frame_unref(frame_);
            return false;
        }
        src_frame = sw_frame_;
    }

    // Initialize buffers if needed (now we know dimensions)
    if (!initSwsContext(src_frame->width, src_frame->height)) {
        av_frame_unref(frame_);
        if (hw_accel_enabled_) av_frame_unref(sw_frame_);
        return false;
    }

    // Use double-buffering: write to current buffer, return it to caller
    // Caller's previous frame (from last decode) is in the other buffer, still valid
    cv::Mat& bgr_buffer = bgr_buffers_[current_buffer_];

    if (hw_accel_enabled_ && src_frame->format == AV_PIX_FMT_NV12) {
        // VAAPI outputs NV12: Y plane + interleaved UV plane
        // Copy to contiguous buffer for OpenCV
        uint8_t* nv12_ptr = nv12_buffer_.data;
        int y_size = width_ * height_;
        int uv_size = width_ * (height_ / 2);  // Interleaved UV

        // Copy Y plane
        if (src_frame->linesize[0] == width_) {
            memcpy(nv12_ptr, src_frame->data[0], y_size);
        } else {
            for (int i = 0; i < height_; i++) {
                memcpy(nv12_ptr + i * width_, src_frame->data[0] + i * src_frame->linesize[0], width_);
            }
        }
        nv12_ptr += y_size;

        // Copy UV plane (interleaved)
        if (src_frame->linesize[1] == width_) {
            memcpy(nv12_ptr, src_frame->data[1], uv_size);
        } else {
            for (int i = 0; i < height_ / 2; i++) {
                memcpy(nv12_ptr + i * width_, src_frame->data[1] + i * src_frame->linesize[1], width_);
            }
        }

        // Convert NV12 to BGR using OpenCV's SIMD-optimized cvtColor
        cv::cvtColor(nv12_buffer_, bgr_buffer, cv::COLOR_YUV2BGR_NV12);
    } else {
        // CPU decode: I420 format (Y + U + V separate planes)
        uint8_t* yuv_ptr = yuv_buffer_.data;
        int y_size = width_ * height_;
        int uv_width = width_ / 2;
        int uv_height = height_ / 2;

        // Copy Y plane (handle stride if different from width)
        if (src_frame->linesize[0] == width_) {
            memcpy(yuv_ptr, src_frame->data[0], y_size);
        } else {
            for (int i = 0; i < height_; i++) {
                memcpy(yuv_ptr + i * width_, src_frame->data[0] + i * src_frame->linesize[0], width_);
            }
        }
        yuv_ptr += y_size;

        // Copy U plane
        if (src_frame->linesize[1] == uv_width) {
            memcpy(yuv_ptr, src_frame->data[1], uv_width * uv_height);
        } else {
            for (int i = 0; i < uv_height; i++) {
                memcpy(yuv_ptr + i * uv_width, src_frame->data[1] + i * src_frame->linesize[1], uv_width);
            }
        }
        yuv_ptr += uv_width * uv_height;

        // Copy V plane
        if (src_frame->linesize[2] == uv_width) {
            memcpy(yuv_ptr, src_frame->data[2], uv_width * uv_height);
        } else {
            for (int i = 0; i < uv_height; i++) {
                memcpy(yuv_ptr + i * uv_width, src_frame->data[2] + i * src_frame->linesize[2], uv_width);
            }
        }

        // Convert YUV I420 to BGR using OpenCV's SIMD-optimized cvtColor
        cv::cvtColor(yuv_buffer_, bgr_buffer, cv::COLOR_YUV2BGR_I420);
    }

    // Release FFmpeg frame data to prevent memory leak
    av_frame_unref(frame_);
    if (hw_accel_enabled_) {
        av_frame_unref(sw_frame_);
    }

    // Return current buffer to caller (zero-copy, caller gets direct reference)
    // Swap to other buffer for next frame
    output = bgr_buffer;
    current_buffer_ = 1 - current_buffer_;

    return true;
}

} // namespace x5_ros_driver
