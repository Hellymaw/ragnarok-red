#ifndef RR_NODE_H
#define RR_NODE_H

#define LED_TASK_PRIOR 10
#define LED_TASK_STACK_SIZE 250

#define RTDOA_NODE_TASK_PRIOR 5
#define RTDOA_NODE_TASK_STACK_SIZE 2048

struct os_task led_task_init, rtdoa_node_task_init;
os_stack_t led_task_stack[LED_TASK_STACK_SIZE];
os_stack_t rtdoa_node_task_stack[RTDOA_NODE_TASK_STACK_SIZE];

void blink_led_task(void *arg);

void rtdoa_node_task(void *arg);

#endif