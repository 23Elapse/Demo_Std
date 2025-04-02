#ifndef __TSK_WIFI_H
#define __TSK_WIFI_H
#include "pch.h"

#define WIFI_TASK_STACK_SIZE 256
#define CMD_QUEUE_SIZE 10

extern QueueHandle_t cmdQueue;

void vWifiTask(void *pvParameters);


#endif
