#ifndef HEARTBEAT_LED_H
#define HEARTBEAT_LED_H

#include "os/os.h"

#define HEARTBEAT_LED_TASK_PRIORITY 10
#define HEARTBEAT_LED_TASK_STACK_SIZE 32

struct os_task heartbeat_led_task_init;
os_stack_t heartbeat_led_task_stack[HEARTBEAT_LED_TASK_STACK_SIZE];

void heartbeat_led_task(void *arg);

#endif