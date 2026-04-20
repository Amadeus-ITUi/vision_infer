#include "zf_common_headfile.h"

#include "uart_thread.h"
#include "vision_thread.h"
#include "driver/config/smartcar_config.h"
#include "driver/vision/vision_config.h"
#include "driver/vision/vision_infer_async.h"
#include "driver/vision/vision_transport.h"
#include "driver/vision/vision_image_processor.h"

#include <string>

volatile sig_atomic_t g_should_exit = 0;

namespace
{
constexpr int kMainLoopPeriodMs = 5;
bool g_worker_cleanup_done = false;

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

    if (!vision_transport_udp_init(g_vision_runtime_config.udp_web_server_ip,
                                   g_vision_runtime_config.udp_web_video_port,
                                   g_vision_runtime_config.udp_web_meta_port))
    {
        printf("[UDP_WEB] init failed\r\n");
    }
    vision_transport_udp_set_enabled(g_vision_runtime_config.udp_web_enabled);
    vision_transport_udp_set_max_fps(g_vision_runtime_config.udp_web_max_fps);
    vision_transport_udp_set_tcp_enabled(g_vision_runtime_config.udp_web_tcp_enabled);

    LQ_NCNN ncnn;
    const bool ncnn_ready = vision_infer_init_default_model(ncnn);
    if (!vision_thread_init("/dev/video0", &ncnn, ncnn_ready))
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

    printf("[VISION] minimal pipeline: bgr -> red_rect -> roi -> ncnn -> web\r\n");
    printf("[UDP_WEB] enabled=%d server=%s video=%u meta=%u fps=%u gray=%d binary=%d rgb=%d\r\n",
           vision_transport_udp_is_enabled() ? 1 : 0,
           g_vision_runtime_config.udp_web_server_ip,
           static_cast<unsigned int>(g_vision_runtime_config.udp_web_video_port),
           static_cast<unsigned int>(g_vision_runtime_config.udp_web_meta_port),
           static_cast<unsigned int>(vision_transport_udp_get_max_fps()),
           g_vision_runtime_config.udp_web_send_gray_jpeg ? 1 : 0,
           g_vision_runtime_config.udp_web_send_binary_jpeg ? 1 : 0,
           g_vision_runtime_config.udp_web_send_rgb_jpeg ? 1 : 0);

    while (!g_should_exit)
    {
        system_delay_ms(kMainLoopPeriodMs);
    }

    printf("收到Ctrl+C,程序即将退出\n");
    cleanup_once();
    return 0;
}
