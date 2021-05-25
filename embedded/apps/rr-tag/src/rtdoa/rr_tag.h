
/**
 ************************************************************************
 * @file rr_tag.h
 * @author Wilfred MK
 * @date 19.05.2021 (Last Updated)
 * @brief Contains required declarations for an rr_tag
 **********************************************************************
 **/


#ifndef RR_TAG_H
#define RR_TAG_H

#include "os/os.h"
#include "rtdoa_backhaul/rtdoa_backhaul.h"

// Priority of RTDoA task
#define RTDOA_TAG_TASK_PRIOR 5

// Stack size of RTDoA task
#define RTDOA_TAG_TASK_STACK_SIZE 256

// Nodes IDS
#define MASTER_NODE_ID 0xc439

// Slave node UWB addresses
#define SLAVE_NODE1_ID 0x8817
#define SLAVE_NODE2_ID 0x28BB
#define SLAVE_NODE3_ID 0xC6B3
#define SLAVE_NODE4_ID 0x5132
#define SLAVE_NODE5_ID 0x256A
#define SLAVE_NODE6_ID 0x19AB

// RTDoA task
struct os_task rtdoa_tag_task_init;

// RTDoA task stack
os_stack_t rtdoa_tag_task_stack[RTDOA_TAG_TASK_STACK_SIZE];

// Task to handle RTDoA tag responsibilities
void rtdoa_tag_task(void *arg);

// Handles received range packets
void rr_tag_packet(struct rtdoabh_tag_results_pkg *p);

#endif