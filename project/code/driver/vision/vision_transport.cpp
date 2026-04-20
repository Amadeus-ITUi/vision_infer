#include "driver/vision/vision_transport.h"

#include "app/vision_thread/vision_thread.h"
#include "driver/vision/vision_config.h"
#include "driver/vision/vision_frame_capture.h"
#include "driver/vision/vision_image_processor.h"
#include "driver/vision/vision_infer_async.h"

#include "zf_driver_tcp_client.h"
#include "zf_driver_udp.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

namespace
{
constexpr uint32 kUdpDefaultMaxFps = 30;
constexpr uint32 kUdpMaxPayload = 1200;
constexpr uint32 kUdpHeaderSize = 20;
constexpr uint32 kMagic = 0x56535544;
constexpr int kGrayJpegQuality = 100;
constexpr int kRgbJpegQuality = 85;
constexpr int kRoi64Size = 64;

#pragma pack(push, 1)
struct udp_chunk_header_t
{
    uint32 magic;
    uint32 frame_id;
    uint16 chunk_idx;
    uint16 chunk_total;
    uint16 payload_len;
    uint16 width;
    uint16 height;
    uint8 mode;
    uint8 reserved;
};
#pragma pack(pop)

std::atomic<uint32> g_last_send_time_us(0);

std::atomic<bool> g_udp_enabled(false);
std::atomic<bool> g_tcp_enabled(true);
std::atomic<uint32> g_udp_max_fps(kUdpDefaultMaxFps);
std::atomic<uint32> g_udp_frame_id(0);
std::atomic<uint32> g_udp_tx_fps(0);
std::atomic<uint64> g_udp_last_send_tick_us(0);

std::mutex g_init_mutex;
bool g_udp_ready = false;
bool g_tcp_ready = false;
char g_server_ip[64] = {0};
uint16 g_video_port = 0;
uint16 g_meta_port = 0;

uint64 now_us()
{
    return static_cast<uint64>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool fps_limited(std::atomic<uint64> &last_tick_us, uint32 max_fps)
{
    if (max_fps == 0)
    {
        return false;
    }
    const uint64 now = now_us();
    const uint64 min_interval_us = 1000000ULL / std::max<uint32>(1, max_fps);
    const uint64 last = last_tick_us.load();
    if (last != 0 && now - last < min_interval_us)
    {
        return true;
    }
    last_tick_us.store(now);
    return false;
}

std::vector<uchar> encode_image(const cv::Mat &image, bool gray)
{
    std::vector<uchar> encoded;
    std::vector<int> params = {
        cv::IMWRITE_JPEG_QUALITY, gray ? kGrayJpegQuality : kRgbJpegQuality
    };
    cv::imencode(".jpg", image, encoded, params);
    return encoded;
}

void udp_send_frame_payload(const std::vector<uchar> &encoded, uint16 width, uint16 height, uint8 mode)
{
    if (!g_udp_ready || encoded.empty())
    {
        return;
    }

    const uint32 frame_id = g_udp_frame_id.fetch_add(1) + 1;
    const uint32 chunk_capacity = kUdpMaxPayload - kUdpHeaderSize;
    const uint16 chunk_total = static_cast<uint16>((encoded.size() + chunk_capacity - 1) / chunk_capacity);
    std::array<uint8, kUdpMaxPayload> packet = {};

    for (uint16 chunk_idx = 0; chunk_idx < chunk_total; ++chunk_idx)
    {
        const size_t offset = static_cast<size_t>(chunk_idx) * chunk_capacity;
        const size_t payload_len = std::min<size_t>(chunk_capacity, encoded.size() - offset);
        udp_chunk_header_t header{};
        header.magic = kMagic;
        header.frame_id = frame_id;
        header.chunk_idx = chunk_idx;
        header.chunk_total = chunk_total;
        header.payload_len = static_cast<uint16>(payload_len);
        header.width = width;
        header.height = height;
        header.mode = mode;
        std::memcpy(packet.data(), &header, sizeof(header));
        std::memcpy(packet.data() + sizeof(header), encoded.data() + offset, payload_len);
        udp_send_data(packet.data(), static_cast<uint32>(sizeof(header) + payload_len));
    }
}

void udp_send_gray_binary_rgb()
{
    if (!g_udp_enabled.load() || fps_limited(g_udp_last_send_tick_us, g_udp_max_fps.load()))
    {
        return;
    }

    uint32 sent_count = 0;

    if (g_vision_runtime_config.udp_web_send_gray_jpeg)
    {
        const uint8 *gray = vision_image_processor_gray_downsampled_image();
        if (gray != nullptr)
        {
            cv::Mat gray_img(VISION_DOWNSAMPLED_HEIGHT, VISION_DOWNSAMPLED_WIDTH, CV_8UC1, const_cast<uint8 *>(gray));
            udp_send_frame_payload(encode_image(gray_img, true), gray_img.cols, gray_img.rows, 1);
            ++sent_count;
        }
    }

    if (g_vision_runtime_config.udp_web_send_binary_jpeg)
    {
        const uint8 *binary = vision_image_processor_binary_downsampled_u8_image();
        if (binary != nullptr)
        {
            cv::Mat binary_img(VISION_DOWNSAMPLED_HEIGHT, VISION_DOWNSAMPLED_WIDTH, CV_8UC1, const_cast<uint8 *>(binary));
            udp_send_frame_payload(encode_image(binary_img, true), binary_img.cols, binary_img.rows, 0);
            ++sent_count;
        }
    }

    if (g_vision_runtime_config.udp_web_send_rgb_jpeg)
    {
        const uint8 *bgr = vision_image_processor_bgr_downsampled_image();
        if (bgr != nullptr)
        {
            cv::Mat bgr_img(VISION_DOWNSAMPLED_HEIGHT, VISION_DOWNSAMPLED_WIDTH, CV_8UC3, const_cast<uint8 *>(bgr));
            udp_send_frame_payload(encode_image(bgr_img, false), bgr_img.cols, bgr_img.rows, 2);
            ++sent_count;
        }
    }

    bool roi_valid = false;
    int roi_x = 0;
    int roi_y = 0;
    int roi_w = 0;
    int roi_h = 0;
    vision_image_processor_get_ncnn_roi(&roi_valid, &roi_x, &roi_y, &roi_w, &roi_h);
    if (roi_valid && roi_w > 0 && roi_h > 0)
    {
        const uint8 *bgr = vision_image_processor_bgr_downsampled_image();
        if (bgr != nullptr)
        {
            cv::Mat bgr_img(VISION_DOWNSAMPLED_HEIGHT, VISION_DOWNSAMPLED_WIDTH, CV_8UC3, const_cast<uint8 *>(bgr));
            cv::Rect roi(roi_x, roi_y, roi_w, roi_h);
            roi &= cv::Rect(0, 0, bgr_img.cols, bgr_img.rows);
            if (roi.width > 0 && roi.height > 0)
            {
                cv::Mat roi_resized;
                cv::resize(bgr_img(roi), roi_resized, cv::Size(kRoi64Size, kRoi64Size), 0.0, 0.0, cv::INTER_LINEAR);
                udp_send_frame_payload(encode_image(roi_resized, false), roi_resized.cols, roi_resized.rows, 3);
                ++sent_count;
            }
        }
    }

    if (sent_count > 0)
    {
        g_udp_tx_fps.store(sent_count);
    }
}

std::string json_escape(const std::string &value)
{
    std::string out;
    out.reserve(value.size() + 8);
    for (char ch : value)
    {
        if (ch == '\\' || ch == '"')
        {
            out.push_back('\\');
        }
        out.push_back(ch);
    }
    return out;
}

void tcp_send_status()
{
    if (!g_tcp_enabled.load() || !g_tcp_ready)
    {
        return;
    }

    uint32 capture_wait_us = 0;
    uint32 preprocess_us = 0;
    uint32 otsu_us = 0;
    uint32 total_us = 0;
    vision_image_processor_get_last_perf_us(&capture_wait_us, &preprocess_us, &otsu_us, nullptr, &total_us);

    bool red_found = false;
    int red_x = 0;
    int red_y = 0;
    int red_w = 0;
    int red_h = 0;
    int red_cx = 0;
    int red_cy = 0;
    vision_image_processor_get_red_rect(&red_found, &red_x, &red_y, &red_w, &red_h, &red_cx, &red_cy);

    bool roi_valid = false;
    int roi_x = 0;
    int roi_y = 0;
    int roi_w = 0;
    int roi_h = 0;
    vision_image_processor_get_ncnn_roi(&roi_valid, &roi_x, &roi_y, &roi_w, &roi_h);

    vision_infer_async_result_t infer_result{};
    const bool has_infer_result = vision_infer_async_fetch_latest(&infer_result);

    char line[4096];
    std::snprintf(line,
                  sizeof(line),
                  "{\"web_data_profile\":%d,"
                  "\"ts_ms\":%llu,"
                  "\"capture_thread_fps\":%u,"
                  "\"vision_process_fps\":%u,"
                  "\"udp_tx_fps\":%u,"
                  "\"otsu_threshold\":%u,"
                  "\"capture_wait_us\":%u,"
                  "\"preprocess_us\":%u,"
                  "\"otsu_us\":%u,"
                  "\"total_us\":%u,"
                  "\"red_found\":%s,"
                  "\"red\":[%d,%d,%d,%d,%d,%d],"
                  "\"roi_valid\":%s,"
                  "\"roi\":[%d,%d,%d,%d],"
                  "\"infer_enabled\":%s,"
                  "\"ncnn_enabled\":%s,"
                  "\"ncnn_has_result\":%s,"
                  "\"ncnn_infer_valid\":%s,"
                  "\"ncnn_infer_us\":%u,"
                  "\"ncnn_top_class_id\":%d,"
                  "\"ncnn_top_score\":%.6f,"
                  "\"ncnn_top_label\":\"%s\","
                  "\"gray_size\":[%d,%d],"
                  "\"roi64_size\":[%d,%d]}\n",
                  g_vision_runtime_config.udp_web_data_profile,
                  static_cast<unsigned long long>(now_us() / 1000ULL),
                  static_cast<unsigned int>(vision_frame_capture_fps()),
                  static_cast<unsigned int>(vision_thread_process_fps()),
                  static_cast<unsigned int>(g_udp_tx_fps.load()),
                  static_cast<unsigned int>(vision_image_processor_get_last_otsu_threshold()),
                  static_cast<unsigned int>(capture_wait_us),
                  static_cast<unsigned int>(preprocess_us),
                  static_cast<unsigned int>(otsu_us),
                  static_cast<unsigned int>(total_us),
                  red_found ? "true" : "false",
                  red_x,
                  red_y,
                  red_w,
                  red_h,
                  red_cx,
                  red_cy,
                  roi_valid ? "true" : "false",
                  roi_x,
                  roi_y,
                  roi_w,
                  roi_h,
                  vision_infer_async_enabled() ? "true" : "false",
                  (has_infer_result ? infer_result.ncnn_enabled : vision_infer_async_ncnn_enabled()) ? "true" : "false",
                  has_infer_result ? "true" : "false",
                  (has_infer_result && infer_result.ncnn_infer_valid) ? "true" : "false",
                  has_infer_result ? infer_result.ncnn_infer_us : 0U,
                  has_infer_result ? infer_result.ncnn_top_class_id : -1,
                  has_infer_result ? infer_result.ncnn_top_score : 0.0f,
                  json_escape((has_infer_result && infer_result.ncnn_infer_valid) ? infer_result.ncnn_top_label : "").c_str(),
                  VISION_DOWNSAMPLED_WIDTH,
                  VISION_DOWNSAMPLED_HEIGHT,
                  kRoi64Size,
                  kRoi64Size);
    tcp_client_send_data(reinterpret_cast<const uint8 *>(line), static_cast<uint32>(std::strlen(line)));
}
} // namespace

void vision_transport_init()
{
    g_last_send_time_us.store(0);
    g_udp_tx_fps.store(0);
}

void vision_transport_send_step()
{
    const uint64 start_us = now_us();
    udp_send_gray_binary_rgb();
    tcp_send_status();
    g_last_send_time_us.store(static_cast<uint32>(now_us() - start_us));
}

uint32 vision_transport_get_last_send_time_us()
{
    return g_last_send_time_us.load();
}

bool vision_transport_udp_init(const char *server_ip, uint16 video_port, uint16 meta_port)
{
    const std::lock_guard<std::mutex> lock(g_init_mutex);
    g_udp_ready = false;
    g_tcp_ready = false;
    g_video_port = video_port;
    g_meta_port = meta_port;

    if (server_ip == nullptr || server_ip[0] == '\0')
    {
        return false;
    }

    std::snprintf(g_server_ip, sizeof(g_server_ip), "%s", server_ip);
    g_udp_ready = (udp_init(g_server_ip, g_video_port) == 0);
    if (g_meta_port != 0)
    {
        g_tcp_ready = (tcp_client_init(g_server_ip, g_meta_port) == 0);
    }
    return g_udp_ready || g_tcp_ready;
}

void vision_transport_udp_cleanup()
{
}

void vision_transport_udp_set_enabled(bool enabled)
{
    g_udp_enabled.store(enabled);
}

bool vision_transport_udp_is_enabled()
{
    return g_udp_enabled.load();
}

void vision_transport_udp_set_max_fps(uint32 max_fps)
{
    g_udp_max_fps.store(max_fps);
}

uint32 vision_transport_udp_get_max_fps()
{
    return g_udp_max_fps.load();
}

void vision_transport_udp_set_tcp_enabled(bool enabled)
{
    g_tcp_enabled.store(enabled);
}

bool vision_transport_udp_tcp_enabled()
{
    return g_tcp_enabled.load();
}
