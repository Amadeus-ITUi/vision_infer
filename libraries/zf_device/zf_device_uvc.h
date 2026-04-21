#ifndef _zf_driver_uvc_h
#define _zf_driver_uvc_h


#include "zf_common_typedef.h"

// UVC采图分辨率配置（构建时由 UVC_RES_PRESET 选择）：
// 0 -> 160x120
// 1 -> 320x240
// 2 -> 640x480
#define UVC_RES_PRESET_160X120 0
#define UVC_RES_PRESET_320X240 1
#define UVC_RES_PRESET_640X480 2

#ifndef UVC_RES_PRESET
#define UVC_RES_PRESET UVC_RES_PRESET_160X120
#endif

#if (UVC_RES_PRESET == UVC_RES_PRESET_160X120)
#define UVC_WIDTH   160
#define UVC_HEIGHT  120
#elif (UVC_RES_PRESET == UVC_RES_PRESET_320X240)
#define UVC_WIDTH   320
#define UVC_HEIGHT  240
#elif (UVC_RES_PRESET == UVC_RES_PRESET_640X480)
#define UVC_WIDTH   640
#define UVC_HEIGHT  480
#else
#error "Invalid UVC_RES_PRESET. Use 160x120/320x240/640x480 presets."
#endif

// UVC采图帧率配置（构建时由 UVC_FPS_PRESET 选择）：
// 0 -> 60
// 1 -> 90
// 2 -> 120
// 3 -> 180
#define UVC_FPS_PRESET_60 0
#define UVC_FPS_PRESET_90 1
#define UVC_FPS_PRESET_120 2
#define UVC_FPS_PRESET_180 3

#ifndef UVC_FPS_PRESET
#define UVC_FPS_PRESET UVC_FPS_PRESET_120
#endif

#if (UVC_FPS_PRESET == UVC_FPS_PRESET_60)
#define UVC_FPS 60
#elif (UVC_FPS_PRESET == UVC_FPS_PRESET_90)
#define UVC_FPS 90
#elif (UVC_FPS_PRESET == UVC_FPS_PRESET_120)
#define UVC_FPS 120
#elif (UVC_FPS_PRESET == UVC_FPS_PRESET_180)
#define UVC_FPS 180
#else
#error "Invalid UVC_FPS_PRESET. Use 60/90/120/180 presets."
#endif

int8 uvc_camera_init(const char *path);
int8 wait_image_refresh();


extern uint8_t *rgay_image;
extern uint8_t *rgb565_image;
extern uint8_t *bgr_image;


#endif
