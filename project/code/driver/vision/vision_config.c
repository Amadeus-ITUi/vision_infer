#include "driver/vision/vision_config.h"

vision_runtime_config_t g_vision_runtime_config = {
    .infer_enabled = true,
    .ncnn_enabled = true,

    .ncnn_input_width = 64,
    .ncnn_input_height = 64,
    .ncnn_label_count = 3,
    .ncnn_labels = {
        "supplies",
        "vehicles",
        "weapons",
    },

    .udp_web_enabled = true,
    .udp_web_max_fps = 10,
    .udp_web_send_gray_jpeg = false,
    .udp_web_gray_image_format = VISION_WEB_IMAGE_FORMAT_JPEG,
    .udp_web_send_binary_jpeg = false,
    .udp_web_binary_image_format = VISION_WEB_IMAGE_FORMAT_JPEG,
    .udp_web_send_rgb_jpeg = true,
    .udp_web_rgb_image_format = VISION_WEB_IMAGE_FORMAT_JPEG,
    .udp_web_data_profile = VISION_WEB_DATA_PROFILE_RAW_MINIMAL,
    .udp_web_tcp_enabled = true,
    .udp_web_server_ip = "172.21.79.179",
    .udp_web_video_port = 10000,
    .udp_web_meta_port = 10001,
};

vision_processor_config_t g_vision_processor_config = {
    .reserved = false,
};
