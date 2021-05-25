/**
 ************************************************************************
 * @file heartbeat_led.h
 * @author Aaron Helmore
 * @date 24.05.2021 (Last Updated)
 * @brief Contains heardbeat task declarations.
 **********************************************************************
 **/

#ifndef HEARTBEAT_LED_H
#define HEARTBEAT_LED_H

#include "os/os.h"

// Priority of heartbeat LED task
#define HEARTBEAT_LED_TASK_PRIORITY 10

// Stack size of heartbeat LED task
#define HEARTBEAT_LED_TASK_STACK_SIZE 32

// Heartbeat LED task
struct os_task heartbeat_led_task_init;

// Heartbeat LED task stack
os_stack_t heartbeat_led_task_stack[HEARTBEAT_LED_TASK_STACK_SIZE];

// Task to blink heartbeat LED at 2Hz
void heartbeat_led_task(void *arg);

#endif