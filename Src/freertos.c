#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

osThreadId defaultTaskHandle;

static void StartDefaultTask(void const *argument);

void MX_FREERTOS_Init(void)
{
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
}

static void StartDefaultTask(void const *argument)
{
    (void)argument;
    MX_USB_DEVICE_Init();
    osThreadTerminate(NULL);

    for (;;)
    {
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}
