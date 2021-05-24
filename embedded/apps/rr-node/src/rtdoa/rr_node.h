#ifndef RR_NODE_H
#define RR_NODE_H

#include "os/os.h"

// Priority of RTDoA task
#define RTDOA_NODE_TASK_PRIORITY 5

// Stack size of RTDoA task
#define RTDOA_NODE_TASK_STACK_SIZE 256

// RTDoA task
struct os_task rtdoa_node_task_init;

// Stack of RTDoA task
os_stack_t rtdoa_node_task_stack[RTDOA_NODE_TASK_STACK_SIZE];

// RTDoA task
void rtdoa_node_task(void *arg);

#endif