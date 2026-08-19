#include "bsp_log.h"

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include <stdio.h>

#define RTT_DASHBOARD_CH 1u
// Dashboard frames are up to 425 bytes and the swerve target emits at 100 Hz.
// Keep several whole frames buffered to tolerate host poll jitter without
// inflating dashboard_drop_count under normal operation.
#define RTT_DASHBOARD_BUF_SIZE 2048u

static char rtt_dashboard_up_buffer[RTT_DASHBOARD_BUF_SIZE];

void BSPLogInit()
{
    SEGGER_RTT_Init();
    SEGGER_RTT_ConfigUpBuffer(RTT_DASHBOARD_CH,
                              "Dashboard",
                              rtt_dashboard_up_buffer,
                              RTT_DASHBOARD_BUF_SIZE,
                              SEGGER_RTT_MODE_NO_BLOCK_SKIP);
}

int PrintLog(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int n = SEGGER_RTT_vprintf(BUFFER_INDEX, fmt, &args); // 一次可以开启多个buffer(多个终端),我们只用一个
    va_end(args);
    return n;
}

void Float2Str(char *str, float va)
{
    int flag = va < 0;
    int head = (int)va;
    int point = (int)((va - head) * 1000);
    head = abs(head);
    point = abs(point);
    if (flag)
        sprintf(str, "-%d.%d", head, point);
    else
        sprintf(str, "%d.%d", head, point);
}
