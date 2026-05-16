#include "zf_common_headfile.h"

#include "uart_thread.h"
#include "vision_thread.h"
#include "driver/config/smartcar_config.h"
#include "driver/vision/vision_config.h"
#include "driver/vision/vision_frame_capture.h"
#include "driver/vision/vision_infer_async.h"
#include "driver/vision/vision_transport.h"
#include "driver/vision/vision_image_processor.h"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <dirent.h>
#include <sys/stat.h>
#include <string>
#include <vector>
#include <unistd.h>
#include <limits.h>

volatile sig_atomic_t g_should_exit = 0;

namespace
{
constexpr int kMainLoopPeriodMs = 5;
bool g_worker_cleanup_done = false;
constexpr int kExpectedOfflineImageWidth = 64;
constexpr int kExpectedOfflineImageHeight = 64;
constexpr size_t kOfflineCpuSampleStartIndex = 4; // 0-based, the 5th image
constexpr size_t kOfflineCpuTailDropCount = 5;
constexpr useconds_t kBaselineCpuSettleUs = 1000000;
constexpr useconds_t kBaselineCpuSampleWindowUs = 1000000;
constexpr uint64_t kCameraRuntimeConsoleLogIntervalUs = 3000000ULL;

const char *camera_mode_name(int mode)
{
    switch (mode)
    {
        case VISION_CAMERA_MODE_CAPTURE_ONLY: return "camera_capture_only";
        case VISION_CAMERA_MODE_RED_ROI: return "camera_red_roi";
        case VISION_CAMERA_MODE_RED_ROI_NCNN: return "camera_red_roi_ncnn";
        default: return "camera_unknown";
    }
}

struct CpuStatSample
{
    uint64_t total = 0;
    uint64_t idle = 0;
    bool valid = false;
};

bool has_supported_image_extension(const std::string &path)
{
    const size_t dot_pos = path.find_last_of('.');
    if (dot_pos == std::string::npos)
    {
        return false;
    }
    const std::string ext = path.substr(dot_pos);
    std::string lowered;
    lowered.reserve(ext.size());
    for (char ch : ext)
    {
        lowered.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
    }
    return lowered == ".png" || lowered == ".jpg" || lowered == ".jpeg" || lowered == ".bmp";
}

std::string file_name_from_path(const std::string &path)
{
    const size_t slash_pos = path.find_last_of('/');
    return (slash_pos == std::string::npos) ? path : path.substr(slash_pos + 1);
}

std::string join_path(const std::string &dir, const std::string &name)
{
    if (dir.empty())
    {
        return name;
    }
    if (dir.back() == '/')
    {
        return dir + name;
    }
    return dir + "/" + name;
}

bool is_directory_path(const std::string &path)
{
    struct stat st = {};
    if (stat(path.c_str(), &st) != 0)
    {
        return false;
    }
    return S_ISDIR(st.st_mode);
}

std::string executable_dir()
{
    char exe_path[PATH_MAX] = {0};
    const ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (len <= 0)
    {
        return "";
    }
    exe_path[len] = '\0';
    std::string full_path(exe_path);
    const size_t slash_pos = full_path.find_last_of('/');
    if (slash_pos == std::string::npos)
    {
        return "";
    }
    return full_path.substr(0, slash_pos);
}

std::string resolve_offline_image_dir()
{
    char cwd_buf[PATH_MAX] = {0};
    const char *cwd = getcwd(cwd_buf, sizeof(cwd_buf));
    const std::array<std::string, 3> candidates = {
        join_path(executable_dir(), "image"),
        cwd ? join_path(std::string(cwd), "image") : std::string(),
        "image"
    };

    for (const auto &candidate : candidates)
    {
        if (!candidate.empty() && is_directory_path(candidate))
        {
            return candidate;
        }
    }
    return {};
}

CpuStatSample sample_cpu_stat()
{
    std::ifstream stat_file("/proc/stat");
    if (!stat_file.is_open())
    {
        return {};
    }

    std::string cpu_label;
    uint64_t user = 0;
    uint64_t nice = 0;
    uint64_t system = 0;
    uint64_t idle = 0;
    uint64_t iowait = 0;
    uint64_t irq = 0;
    uint64_t softirq = 0;
    uint64_t steal = 0;
    stat_file >> cpu_label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
    if (cpu_label != "cpu")
    {
        return {};
    }

    CpuStatSample sample;
    sample.total = user + nice + system + idle + iowait + irq + softirq + steal;
    sample.idle = idle + iowait;
    sample.valid = true;
    return sample;
}

double compute_cpu_usage_percent(const CpuStatSample &begin, const CpuStatSample &end)
{
    if (!begin.valid || !end.valid || end.total <= begin.total || end.idle < begin.idle)
    {
        return 0.0;
    }

    const uint64_t total_delta = end.total - begin.total;
    const uint64_t idle_delta = end.idle - begin.idle;
    if (total_delta == 0)
    {
        return 0.0;
    }

    const double busy_ratio = static_cast<double>(total_delta - idle_delta) / static_cast<double>(total_delta);
    const double usage_percent = busy_ratio * 100.0;
    return usage_percent < 0.0 ? 0.0 : usage_percent;
}

double sample_cpu_usage_percent_window(useconds_t window_us)
{
    const CpuStatSample begin = sample_cpu_stat();
    if (!begin.valid)
    {
        return 0.0;
    }
    usleep(window_us);
    const CpuStatSample end = sample_cpu_stat();
    return compute_cpu_usage_percent(begin, end);
}

std::string extract_expected_label_from_filename(const std::string &path)
{
    const std::string file_name = file_name_from_path(path);
    const size_t sep_pos = file_name.find("__");
    if (sep_pos == std::string::npos)
    {
        return "";
    }
    return file_name.substr(0, sep_pos);
}

struct OfflineInferRecord
{
    std::string file_name;
    std::string expected_label;
    std::string predicted_label;
    float score = 0.0f;
    uint32 infer_us = 0;
    double cpu_usage_percent = 0.0;
    bool correct = false;
};

int run_offline_image_infer(LQ_NCNN &ncnn)
{
    const std::string image_dir = resolve_offline_image_dir();
    if (image_dir.empty())
    {
        printf("[OFFLINE_INFER] image directory not found. Tried: <exe>/image, ./image, image\r\n");
        return -1;
    }

    std::vector<std::string> image_paths;
    DIR *dir = opendir(image_dir.c_str());
    if (dir == nullptr)
    {
        printf("[OFFLINE_INFER] failed to open image directory: %s\r\n", image_dir.c_str());
        return -1;
    }

    for (;;)
    {
        dirent *entry = readdir(dir);
        if (entry == nullptr)
        {
            break;
        }

        const std::string name(entry->d_name);
        if (name == "." || name == "..")
        {
            continue;
        }

        const std::string full_path = join_path(image_dir, name);
        struct stat st = {};
        if (stat(full_path.c_str(), &st) != 0 || !S_ISREG(st.st_mode))
        {
            continue;
        }
        if (has_supported_image_extension(name))
        {
            image_paths.push_back(full_path);
        }
    }
    closedir(dir);

    std::sort(image_paths.begin(), image_paths.end());
    if (image_paths.empty())
    {
        printf("[OFFLINE_INFER] no supported images found in %s\r\n", image_dir.c_str());
        return -1;
    }

    printf("[OFFLINE_INFER] mode enabled, image_dir=%s, image_count=%u\r\n",
           image_dir.c_str(),
           static_cast<unsigned int>(image_paths.size()));
    printf("[OFFLINE_INFER] 等待基线CPU稳定中...\r\n");
    usleep(kBaselineCpuSettleUs);
    const double baseline_cpu_usage_percent = sample_cpu_usage_percent_window(kBaselineCpuSampleWindowUs);
    printf("[OFFLINE_INFER] 基线CPU占用率=%6.2f%%\r\n", baseline_cpu_usage_percent);

    size_t processed_count = 0;
    std::vector<OfflineInferRecord> records;
    records.reserve(image_paths.size());
    for (const auto &image_path : image_paths)
    {
        cv::Mat bgr = cv::imread(image_path, cv::IMREAD_COLOR);
        if (bgr.empty())
        {
            printf("[OFFLINE_INFER] skip unreadable image: %s\r\n", file_name_from_path(image_path).c_str());
            continue;
        }
        if (bgr.cols != kExpectedOfflineImageWidth || bgr.rows != kExpectedOfflineImageHeight)
        {
            printf("[OFFLINE_INFER] skip image with invalid size: %s (got=%dx%d expected=%dx%d)\r\n",
                   file_name_from_path(image_path).c_str(),
                   bgr.cols,
                   bgr.rows,
                   kExpectedOfflineImageWidth,
                   kExpectedOfflineImageHeight);
            continue;
        }

        const std::string expected_label = extract_expected_label_from_filename(image_path);
        const CpuStatSample cpu_begin = sample_cpu_stat();
        int top_class_id = -1;
        float top_score = 0.0f;
        uint32 infer_us = 0;
        std::string top_label;
        std::vector<std::string> labels;
        std::vector<float> probs;
        bool ok = false;
        try
        {
            ok = ncnn.InferWithProbs(bgr,
                                     &top_class_id,
                                     &top_score,
                                     &top_label,
                                     &labels,
                                     &probs,
                                     &infer_us);
        }
        catch (const std::exception &ex)
        {
            printf("[OFFLINE_INFER] infer failed: %s (%s)\r\n",
                   file_name_from_path(image_path).c_str(),
                   ex.what());
            continue;
        }

        const CpuStatSample cpu_end = sample_cpu_stat();
        const double cpu_usage_percent = compute_cpu_usage_percent(cpu_begin, cpu_end);
        if (!ok)
        {
            printf("[OFFLINE_INFER] infer returned false: %s\r\n", file_name_from_path(image_path).c_str());
            continue;
        }

        printf("[OFFLINE_INFER] 结果=%-12s  置信度=%7.4f  耗时=%7.3fms\r\n",
               top_label.c_str(),
               static_cast<double>(top_score),
               static_cast<double>(infer_us) / 1000.0);
        ++processed_count;
        OfflineInferRecord record{};
        record.file_name = file_name_from_path(image_path);
        record.expected_label = expected_label;
        record.predicted_label = top_label;
        record.score = top_score;
        record.infer_us = infer_us;
        record.cpu_usage_percent = cpu_usage_percent;
        record.correct = !expected_label.empty() && expected_label == top_label;
        records.push_back(record);
    }

    if (processed_count == 0)
    {
        printf("[OFFLINE_INFER] no valid images were inferred successfully\r\n");
        return -1;
    }

    if (records.size() < (kOfflineCpuSampleStartIndex + kOfflineCpuTailDropCount + 1))
    {
        printf("[OFFLINE_INFER] 有效推理样本不足，无法计算中段CPU均值: valid_count=%u\r\n",
               static_cast<unsigned int>(records.size()));
        return -1;
    }

    const size_t cpu_sample_end_exclusive = records.size() - kOfflineCpuTailDropCount;
    double mid_cpu_usage_sum = 0.0;
    size_t mid_cpu_usage_count = 0;
    for (size_t i = kOfflineCpuSampleStartIndex; i < cpu_sample_end_exclusive; ++i)
    {
        mid_cpu_usage_sum += records[i].cpu_usage_percent;
        ++mid_cpu_usage_count;
    }
    const double mid_cpu_usage_mean = (mid_cpu_usage_count > 0)
                                          ? (mid_cpu_usage_sum / static_cast<double>(mid_cpu_usage_count))
                                          : 0.0;
    const double infer_cpu_usage_percent = std::max(0.0, mid_cpu_usage_mean - baseline_cpu_usage_percent);

    size_t correct_count = 0;
    for (const auto &record : records)
    {
        if (record.correct)
        {
            ++correct_count;
        }
    }
    const double accuracy_percent = records.empty()
                                        ? 0.0
                                        : (static_cast<double>(correct_count) * 100.0 / static_cast<double>(records.size()));

    printf("[OFFLINE_INFER] 中段推理CPU均值=%6.2f%%\r\n", mid_cpu_usage_mean);
    printf("[OFFLINE_INFER] 推理所需CPU占用率=%6.2f%%\r\n", infer_cpu_usage_percent);
    printf("[OFFLINE_INFER] 准确率=%u/%u (%.2f%%)\r\n",
           static_cast<unsigned int>(correct_count),
           static_cast<unsigned int>(records.size()),
           accuracy_percent);

    if (g_vision_runtime_config.offline_image_accuracy_report_mode == 2)
    {
        bool has_wrong_case = false;
        for (const auto &record : records)
        {
            if (record.correct)
            {
                continue;
            }
            if (!has_wrong_case)
            {
                printf("[OFFLINE_INFER] 错例列表:\r\n");
                has_wrong_case = true;
            }
            printf("[OFFLINE_INFER]   文件=%s  真值=%s  预测=%s  置信度=%.4f\r\n",
                   record.file_name.c_str(),
                   record.expected_label.empty() ? "<unknown>" : record.expected_label.c_str(),
                   record.predicted_label.c_str(),
                   static_cast<double>(record.score));
        }
        if (!has_wrong_case)
        {
            printf("[OFFLINE_INFER] 错例列表: 无\r\n");
        }
    }

    printf("[OFFLINE_INFER] all images processed, exit now\r\n");
    return 0;
}

void cleanup_worker_threads_once()
{
    if (g_worker_cleanup_done)
    {
        return;
    }

    vision_thread_cleanup();
    vision_transport_udp_cleanup();
    uart_thread_cleanup();
    g_worker_cleanup_done = true;
}

void cleanup_once()
{
    cleanup_worker_threads_once();
}
} // namespace

void exit_signal_handler(int signum)
{
    (void)signum;
    g_should_exit = 1;
}

void cleanup()
{
    cleanup_once();
}

int main(int, char **)
{
    signal(SIGINT, exit_signal_handler);
    signal(SIGTERM, exit_signal_handler);
    signal(SIGQUIT, exit_signal_handler);
    signal(SIGHUP, exit_signal_handler);

    std::string loaded_config_path;
    std::string config_error_message;
    if (!smartcar_config_load_from_default_locations(&loaded_config_path, &config_error_message))
    {
        printf("[CONFIG] load failed: %s\r\n", config_error_message.c_str());
        cleanup_once();
        return -1;
    }
    vision_image_processor_reload_config_from_globals();
    printf("[CONFIG] loaded=%s\r\n", loaded_config_path.c_str());

    const bool need_ncnn =
        g_vision_runtime_config.offline_image_infer_mode ||
        g_vision_runtime_config.camera_mode == VISION_CAMERA_MODE_RED_ROI_NCNN;
    LQ_NCNN ncnn;
    bool ncnn_ready = false;
    if (need_ncnn)
    {
        ncnn_ready = vision_infer_init_default_model(ncnn);
        if (!ncnn_ready)
        {
            cleanup_once();
            return -1;
        }
    }

    if (g_vision_runtime_config.offline_image_infer_mode)
    {
        const int ret = run_offline_image_infer(ncnn);
        cleanup_once();
        return ret;
    }

    if (g_vision_runtime_config.udp_web_enabled)
    {
        if (!vision_transport_udp_init(g_vision_runtime_config.udp_web_server_ip,
                                       g_vision_runtime_config.udp_web_video_port,
                                       g_vision_runtime_config.udp_web_meta_port))
        {
            printf("[UDP_WEB] init failed\r\n");
        }
    }
    vision_transport_udp_set_enabled(g_vision_runtime_config.udp_web_enabled);
    vision_transport_udp_set_max_fps(g_vision_runtime_config.udp_web_max_fps);
    vision_transport_udp_set_tcp_enabled(g_vision_runtime_config.udp_web_enabled &&
                                         g_vision_runtime_config.udp_web_tcp_enabled);

    if (!vision_thread_init("/dev/video0", need_ncnn ? &ncnn : nullptr, ncnn_ready))
    {
        cleanup();
        return -1;
    }

    vision_thread_set_infer_enabled(g_vision_runtime_config.infer_enabled);
    vision_thread_set_ncnn_enabled(g_vision_runtime_config.ncnn_enabled);

    if (!uart_thread_init())
    {
        cleanup();
        return -1;
    }

    printf("[VISION] camera_mode=%s offline=%d web=%d red=%d roi=%d ncnn=%d\r\n",
           camera_mode_name(g_vision_runtime_config.camera_mode),
           g_vision_runtime_config.offline_image_infer_mode ? 1 : 0,
           g_vision_runtime_config.udp_web_enabled ? 1 : 0,
           g_vision_runtime_config.red_detect_enabled ? 1 : 0,
           g_vision_runtime_config.roi_draw_enabled ? 1 : 0,
           g_vision_runtime_config.ncnn_enabled ? 1 : 0);
    printf("[UDP_WEB] enabled=%d server=%s video=%u meta=%u fps=%u gray=%d rgb=%d\r\n",
           vision_transport_udp_is_enabled() ? 1 : 0,
           g_vision_runtime_config.udp_web_server_ip,
           static_cast<unsigned int>(g_vision_runtime_config.udp_web_video_port),
           static_cast<unsigned int>(g_vision_runtime_config.udp_web_meta_port),
           static_cast<unsigned int>(vision_transport_udp_get_max_fps()),
           g_vision_runtime_config.udp_web_send_gray_jpeg ? 1 : 0,
           g_vision_runtime_config.udp_web_send_rgb_jpeg ? 1 : 0);

    (void)vision_transport_read_console_cpu_usage_percent();
    auto last_runtime_log = std::chrono::steady_clock::now();
    while (!g_should_exit)
    {
        const auto now = std::chrono::steady_clock::now();
        const uint64_t elapsed_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(now - last_runtime_log).count());
        if (elapsed_us >= kCameraRuntimeConsoleLogIntervalUs)
        {
            last_runtime_log = now;
            const double cpu_usage_percent = vision_transport_read_console_cpu_usage_percent();
            printf("[CAMERA_RUNTIME] mode=%s cpu=%6.2f%% capture_fps=%u process_fps=%u udp_tx_fps=%u infer_fps=%u\r\n",
                   camera_mode_name(g_vision_runtime_config.camera_mode),
                   cpu_usage_percent,
                   static_cast<unsigned int>(vision_frame_capture_fps()),
                   static_cast<unsigned int>(vision_thread_process_fps()),
                   static_cast<unsigned int>(vision_transport_get_udp_tx_fps()),
                   static_cast<unsigned int>(vision_infer_async_fps()));
        }
        system_delay_ms(kMainLoopPeriodMs);
    }

    printf("收到Ctrl+C,程序即将退出\n");
    cleanup_once();
    return 0;
}
