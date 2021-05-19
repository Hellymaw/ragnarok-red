#ifndef RR_NODE_H
#define RR_NODE_H

#define LED_TASK_PRIOR 10
#define LED_TASK_STACK_SIZE 250

struct os_task led_task_init;
os_stack_t led_task_stack[LED_TASK_STACK_SIZE];

void blink_led_task(void *arg);

#endif