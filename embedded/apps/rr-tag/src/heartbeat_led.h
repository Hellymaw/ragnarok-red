#ifndef HEARTBEAT_LED_H
#define HEARTBEAT_LED_H

#include "os/os.h"
#include "rtdoa_backhaul/rtdoa_backhaul.h"

#define HEART_LED_TASK_PRIORITY 10
#define HEART_LED_TASK_STACK_SIZE 32

struct os_task heartbeat_led_task_init;
os_stack_t heartbeat_led_task_stack[HEART_LED_TASK_STACK_SIZE];

void heartbeat_led_task(void *arg);

#endif