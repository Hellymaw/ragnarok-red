#ifndef RR_NODE_H
#define RR_NODE_H

#include "os/os.h"

#define RTDOA_NODE_TASK_PRIORITY 5
#define RTDOA_NODE_TASK_STACK_SIZE 2048

struct os_task rtdoa_node_task_init;
os_stack_t rtdoa_node_task_stack[RTDOA_NODE_TASK_STACK_SIZE];


void rtdoa_node_task(void *arg);

#endif