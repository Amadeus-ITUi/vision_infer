#ifndef VISION_IMAGE_PROCESSOR_H_
#define VISION_IMAGE_PROCESSOR_H_

#include "zf_common_headfile.h"

#include <cstddef>

// 处理分辨率直接跟随当前采图分辨率，不再额外降采样。
#define VISION_DOWNSAMPLED_WIDTH UVC_WIDTH
#define VISION_DOWNSAMPLED_HEIGHT UVC_HEIGHT
#define VISION_IPM_WIDTH 280
#define VISION_IPM_HEIGHT 140

extern int line_error;
extern float line_sample_ratio;

typedef enum
{
    VISION_IPM_LINE_ERROR_FROM_LEFT_SHIFT = 0,
    VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT = 1,
    VISION_IPM_LINE_ERROR_FROM_AUTO = 2
} vision_ipm_line_error_source_enum;

typedef enum
{
    VISION_IPM_LINE_ERROR_FIXED_INDEX = 0,
    VISION_IPM_LINE_ERROR_WEIGHTED_INDEX = 1,
    VISION_IPM_LINE_ERROR_SPEED_INDEX = 2,
    VISION_IPM_LINE_ERROR_WEIGHTED_SPEED_DELTA = 3
} vision_ipm_line_error_method_enum;

bool vision_image_processor_init(const char *camera_path);
void vision_image_processor_cleanup();
bool vision_image_processor_process_step();
uint32 vision_image_processor_processed_frame_seq();
void vision_image_processor_reload_config_from_globals();

const uint8 *vision_image_processor_gray_image();
const uint8 *vision_image_processor_bgr_image();
const uint8 *vision_image_processor_bgr_full_image();
const uint8 *vision_image_processor_gray_downsampled_image();
const uint8 *vision_image_processor_bgr_downsampled_image();

void vision_image_processor_get_last_perf_us(uint32 *capture_wait_us,
                                             uint32 *preprocess_us,
                                             uint32 *maze_us,
                                             uint32 *total_us);
void vision_image_processor_get_last_red_detect_us(uint32 *red_detect_us);
void vision_image_processor_set_last_red_detect_us(uint32 red_detect_us);
void vision_image_processor_get_last_maze_detail_us(uint32 *maze_setup_us,
                                                    uint32 *maze_start_us,
                                                    uint32 *maze_trace_left_us,
                                                    uint32 *maze_trace_right_us,
                                                    uint32 *maze_post_us,
                                                    uint16 *left_points,
                                                    uint16 *right_points,
                                                    bool *left_ok,
                                                    bool *right_ok);

void vision_image_processor_set_red_rect(bool found,
                                         int x,
                                         int y,
                                         int w,
                                         int h,
                                         int cx,
                                         int cy,
                                         int area);
void vision_image_processor_get_red_rect(bool *found,
                                         int *x,
                                         int *y,
                                         int *w,
                                         int *h,
                                         int *cx,
                                         int *cy);
int vision_image_processor_get_red_rect_area();

void vision_image_processor_set_ncnn_roi(bool valid, int x, int y, int w, int h);
void vision_image_processor_get_ncnn_roi(bool *valid, int *x, int *y, int *w, int *h);

#endif
