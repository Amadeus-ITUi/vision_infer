#include "uart_thread.h"

#include "uart.h"

#include <atomic>
#include <chrono>
#include <thread>

namespace
{
Uart g_uart(UART_DEFAULT_DEVICE, UART_DEFAULT_BAUDRATE);
std::thread g_status_thread;
std::atomic<bool> g_status_running(false);

void status_push_loop()
{
    while (g_status_running.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
} // namespace

bool uart_thread_init()
{
    if (g_status_running.load())
    {
        return true;
    }

    if (!g_uart.init())
    {
        printf("uart init failed\r\n");
        return false;
    }

    g_status_running = true;
    g_status_thread = std::thread(status_push_loop);
    return true;
}

void uart_thread_cleanup()
{
    g_status_running = false;

    if (g_status_thread.joinable())
    {
        g_status_thread.join();
    }

    g_uart.close();
}

bool uart_thread_send_text(const char *text)
{
    if (text == nullptr || !g_uart.is_open())
    {
        return false;
    }

    return g_uart.send_string(text) >= 0;
}
