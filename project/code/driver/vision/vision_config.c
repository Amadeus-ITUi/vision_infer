#include "driver/vision/vision_config.h"

vision_runtime_config_t g_vision_runtime_config = {
    .offline_image_infer_mode = false,
    .offline_image_accuracy_report_mode = 1,
    .camera_mode = VISION_CAMERA_MODE_RED_ROI_NCNN,
    .capture_enabled = true,
    .red_detect_enabled = true,
    .roi_draw_enabled = true,
    .infer_enabled = true,
    .ncnn_enabled = true,
    .hsv_debug_enabled = true,

    .ncnn_input_width = 64,
    .ncnn_input_height = 64,
    .ncnn_label_count = 6,
    .ncnn_labels = {
        "ambulance",
        "armored_car",
        "bomb",
        "gun",
        "medicine",
        "telescope",
    },

    .udp_web_enabled = true,
    .udp_web_max_fps = 10,
    .udp_web_send_gray_jpeg = false,
    .udp_web_gray_image_format = VISION_WEB_IMAGE_FORMAT_JPEG,
    .udp_web_send_rgb_jpeg = true,
    .udp_web_rgb_image_format = VISION_WEB_IMAGE_FORMAT_JPEG,
    .udp_web_data_profile = VISION_WEB_DATA_PROFILE_RAW_MINIMAL,
    .udp_web_tcp_enabled = true,
    .udp_web_server_ip = "172.21.79.179",
    .udp_web_video_port = 10000,
    .udp_web_meta_port = 10001,

    .red_search_x_min_permille = 0,
    .red_search_x_max_permille = 1000,
    .red_search_y_min_permille = 200,
    .red_search_y_max_permille = 1000,

    .red_roi_h_span = 12,
    .red_roi_s_min = 50,
    .red_roi_v_min = 50,
    .red_roi_close_iter = 1,
    .red_roi_open_iter = 1,
    .red_roi_area_min = 50,
    .red_roi_ratio_w = 1.2f,
    .red_roi_ratio_h = 1.2f,
    .red_roi_offset_ratio = 0.0f,
};

vision_processor_config_t g_vision_processor_config = {
    .reserved = false,
};
