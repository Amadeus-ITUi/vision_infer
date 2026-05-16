#ifndef VISION_CONFIG_H_
#define VISION_CONFIG_H_

#include "driver/vision/vision_image_processor.h"

#include <stddef.h>

#define VISION_NCNN_CONFIG_MAX_LABELS 16

typedef enum
{
    VISION_WEB_DATA_PROFILE_FULL = 0,
    VISION_WEB_DATA_PROFILE_RAW_MINIMAL = 1
} vision_web_data_profile_enum;

typedef enum
{
    VISION_WEB_IMAGE_FORMAT_JPEG = 0,
    VISION_WEB_IMAGE_FORMAT_PNG = 1,
    VISION_WEB_IMAGE_FORMAT_BMP = 2
} vision_web_image_format_enum;

typedef struct
{
    // 推理运行控制。
    bool offline_image_infer_mode;
    int offline_image_accuracy_report_mode;
    bool infer_enabled;
    bool ncnn_enabled;

    // ncnn 输入参数。
    int ncnn_input_width;
    int ncnn_input_height;
    size_t ncnn_label_count;
    const char *ncnn_labels[VISION_NCNN_CONFIG_MAX_LABELS];

    // 网页发送参数。
    bool udp_web_enabled;
    uint32 udp_web_max_fps;
    bool udp_web_send_gray_jpeg;
    int udp_web_gray_image_format;
    bool udp_web_send_rgb_jpeg;
    int udp_web_rgb_image_format;
    int udp_web_data_profile;
    bool udp_web_tcp_enabled;
    const char *udp_web_server_ip;
    uint16 udp_web_video_port;
    uint16 udp_web_meta_port;

    // 红框检测参数：搜索区（千分比，基于处理图宽高）。
    int red_search_x_min_permille;
    int red_search_x_max_permille;
    int red_search_y_min_permille;
    int red_search_y_max_permille;

    // 红框检测参数：HSV 双区间（仅红色）。
    int red_roi_h_span;
    int red_roi_s_min;
    int red_roi_v_min;
    int red_roi_close_iter;
    int red_roi_open_iter;
    int red_roi_area_min;

    // 红框 -> ncnn ROI 裁剪参数。
    float red_roi_ratio_w;
    float red_roi_ratio_h;
    float red_roi_offset_ratio;
} vision_runtime_config_t;

typedef struct
{
    bool reserved;
} vision_processor_config_t;

extern vision_runtime_config_t g_vision_runtime_config;
extern vision_processor_config_t g_vision_processor_config;

#endif
