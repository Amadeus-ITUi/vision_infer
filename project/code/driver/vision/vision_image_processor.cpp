#include "driver/vision/vision_image_processor.h"

#include "driver/vision/vision_config.h"
#include "driver/vision/vision_frame_capture.h"

#include <opencv2/opencv.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>

namespace
{
constexpr int kProcWidth = VISION_DOWNSAMPLED_WIDTH;
constexpr int kProcHeight = VISION_DOWNSAMPLED_HEIGHT;
constexpr uint32 kCaptureWaitTimeoutMs = 200;

std::array<uint8, UVC_WIDTH * UVC_HEIGHT * 3> g_bgr_full = {};
std::array<uint8, kProcWidth * kProcHeight * 3> g_bgr_proc = {};
std::array<uint8, kProcWidth * kProcHeight> g_gray_proc = {};

std::atomic<uint32> g_processed_frame_seq(0);
std::atomic<uint32> g_last_capture_wait_us(0);
std::atomic<uint32> g_last_preprocess_us(0);
std::atomic<uint32> g_last_total_us(0);
std::atomic<uint32> g_last_red_detect_us(0);

std::mutex g_state_mutex;
bool g_red_found = false;
int g_red_x = 0;
int g_red_y = 0;
int g_red_w = 0;
int g_red_h = 0;
int g_red_cx = 0;
int g_red_cy = 0;
int g_red_area = 0;
bool g_roi_valid = false;
int g_roi_x = 0;
int g_roi_y = 0;
int g_roi_w = 0;
int g_roi_h = 0;
} // namespace

int line_error = 0;
float line_sample_ratio = 0.0f;

bool vision_image_processor_init(const char *camera_path)
{
    g_processed_frame_seq.store(0);
    g_last_capture_wait_us.store(0);
    g_last_preprocess_us.store(0);
    g_last_total_us.store(0);
    g_last_red_detect_us.store(0);
    vision_image_processor_set_red_rect(false, 0, 0, 0, 0, 0, 0, 0);
    vision_image_processor_set_ncnn_roi(false, 0, 0, 0, 0);
    return vision_frame_capture_init(camera_path);
}

void vision_image_processor_cleanup()
{
    vision_frame_capture_cleanup();
}

bool vision_image_processor_process_step()
{
    const auto t_begin = std::chrono::steady_clock::now();

    uint32 capture_wait_us = 0;
    if (!vision_frame_capture_wait_next_bgr(g_bgr_full.data(),
                                            g_bgr_full.size(),
                                            kCaptureWaitTimeoutMs,
                                            &capture_wait_us))
    {
        g_last_capture_wait_us.store(capture_wait_us);
        return false;
    }

    const auto t_pre_begin = std::chrono::steady_clock::now();
    if (g_vision_runtime_config.camera_mode != VISION_CAMERA_MODE_CAPTURE_ONLY)
    {
        std::memcpy(g_bgr_proc.data(), g_bgr_full.data(), g_bgr_proc.size());
        cv::Mat proc(kProcHeight, kProcWidth, CV_8UC3, g_bgr_proc.data());

        cv::Mat gray(kProcHeight, kProcWidth, CV_8UC1, g_gray_proc.data());
        cv::cvtColor(proc, gray, cv::COLOR_BGR2GRAY);
    }
    const auto t_pre_end = std::chrono::steady_clock::now();

    g_last_capture_wait_us.store(capture_wait_us);
    g_last_preprocess_us.store(static_cast<uint32>(
        std::chrono::duration_cast<std::chrono::microseconds>(t_pre_end - t_pre_begin).count()));
    g_last_total_us.store(static_cast<uint32>(
        std::chrono::duration_cast<std::chrono::microseconds>(t_pre_end - t_begin).count()));
    g_processed_frame_seq.fetch_add(1);
    return true;
}

uint32 vision_image_processor_processed_frame_seq()
{
    return g_processed_frame_seq.load();
}

void vision_image_processor_reload_config_from_globals()
{
    line_error = 0;
    line_sample_ratio = 0.0f;
}

const uint8 *vision_image_processor_gray_image()
{
    if (g_vision_runtime_config.camera_mode == VISION_CAMERA_MODE_CAPTURE_ONLY)
    {
        return nullptr;
    }
    return g_gray_proc.data();
}

const uint8 *vision_image_processor_bgr_image()
{
    if (g_vision_runtime_config.camera_mode == VISION_CAMERA_MODE_CAPTURE_ONLY)
    {
        return g_bgr_full.data();
    }
    return g_bgr_proc.data();
}

const uint8 *vision_image_processor_bgr_full_image()
{
    return g_bgr_full.data();
}

const uint8 *vision_image_processor_gray_downsampled_image()
{
    if (g_vision_runtime_config.camera_mode == VISION_CAMERA_MODE_CAPTURE_ONLY)
    {
        return nullptr;
    }
    return g_gray_proc.data();
}

const uint8 *vision_image_processor_bgr_downsampled_image()
{
    if (g_vision_runtime_config.camera_mode == VISION_CAMERA_MODE_CAPTURE_ONLY)
    {
        return g_bgr_full.data();
    }
    return g_bgr_proc.data();
}

void vision_image_processor_get_last_perf_us(uint32 *capture_wait_us,
                                             uint32 *preprocess_us,
                                             uint32 *maze_us,
                                             uint32 *total_us)
{
    if (capture_wait_us != nullptr) *capture_wait_us = g_last_capture_wait_us.load();
    if (preprocess_us != nullptr) *preprocess_us = g_last_preprocess_us.load();
    if (maze_us != nullptr) *maze_us = 0;
    if (total_us != nullptr) *total_us = g_last_total_us.load();
}

void vision_image_processor_get_last_red_detect_us(uint32 *red_detect_us)
{
    if (red_detect_us != nullptr)
    {
        *red_detect_us = g_last_red_detect_us.load();
    }
}

void vision_image_processor_set_last_red_detect_us(uint32 red_detect_us)
{
    g_last_red_detect_us.store(red_detect_us);
}

void vision_image_processor_get_last_maze_detail_us(uint32 *maze_setup_us,
                                                    uint32 *maze_start_us,
                                                    uint32 *maze_trace_left_us,
                                                    uint32 *maze_trace_right_us,
                                                    uint32 *maze_post_us,
                                                    uint16 *left_points,
                                                    uint16 *right_points,
                                                    bool *left_ok,
                                                    bool *right_ok)
{
    if (maze_setup_us != nullptr) *maze_setup_us = 0;
    if (maze_start_us != nullptr) *maze_start_us = 0;
    if (maze_trace_left_us != nullptr) *maze_trace_left_us = 0;
    if (maze_trace_right_us != nullptr) *maze_trace_right_us = 0;
    if (maze_post_us != nullptr) *maze_post_us = 0;
    if (left_points != nullptr) *left_points = 0;
    if (right_points != nullptr) *right_points = 0;
    if (left_ok != nullptr) *left_ok = false;
    if (right_ok != nullptr) *right_ok = false;
}

void vision_image_processor_set_red_rect(bool found,
                                         int x,
                                         int y,
                                         int w,
                                         int h,
                                         int cx,
                                         int cy,
                                         int area)
{
    const std::lock_guard<std::mutex> lock(g_state_mutex);
    g_red_found = found;
    g_red_x = x;
    g_red_y = y;
    g_red_w = w;
    g_red_h = h;
    g_red_cx = cx;
    g_red_cy = cy;
    g_red_area = area;
}

void vision_image_processor_get_red_rect(bool *found,
                                         int *x,
                                         int *y,
                                         int *w,
                                         int *h,
                                         int *cx,
                                         int *cy)
{
    const std::lock_guard<std::mutex> lock(g_state_mutex);
    if (found != nullptr) *found = g_red_found;
    if (x != nullptr) *x = g_red_x;
    if (y != nullptr) *y = g_red_y;
    if (w != nullptr) *w = g_red_w;
    if (h != nullptr) *h = g_red_h;
    if (cx != nullptr) *cx = g_red_cx;
    if (cy != nullptr) *cy = g_red_cy;
}

int vision_image_processor_get_red_rect_area()
{
    const std::lock_guard<std::mutex> lock(g_state_mutex);
    return g_red_area;
}

void vision_image_processor_set_ncnn_roi(bool valid, int x, int y, int w, int h)
{
    const std::lock_guard<std::mutex> lock(g_state_mutex);
    g_roi_valid = valid;
    g_roi_x = x;
    g_roi_y = y;
    g_roi_w = w;
    g_roi_h = h;
}

void vision_image_processor_get_ncnn_roi(bool *valid, int *x, int *y, int *w, int *h)
{
    const std::lock_guard<std::mutex> lock(g_state_mutex);
    if (valid != nullptr) *valid = g_roi_valid;
    if (x != nullptr) *x = g_roi_x;
    if (y != nullptr) *y = g_roi_y;
    if (w != nullptr) *w = g_roi_w;
    if (h != nullptr) *h = g_roi_h;
}
