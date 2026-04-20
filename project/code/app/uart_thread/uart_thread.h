#ifndef __UART_THREAD_H__
#define __UART_THREAD_H__

// 使能上位机调参功能的宏：1为开启，0为关闭
#define ENABLE_UART_TUNING 0

/**
 * @brief 初始化串口通信线程
 * @return 初始化成功返回 true，失败返回 false
 */
bool uart_thread_init();

/**
 * @brief 释放串口通信线程及相关资源
 */
void uart_thread_cleanup();

/**
 * @brief 发送一段文本到串口
 * @return 串口已打开且发送成功时返回 true
 */
bool uart_thread_send_text(const char *text);

#endif
