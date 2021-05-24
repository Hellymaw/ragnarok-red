#ifndef RR_TAG_H
#define RR_TAG_H

#include "os/os.h"
#include "rtdoa_backhaul/rtdoa_backhaul.h"

#define RTDOA_TAG_TASK_PRIOR 5
#define RTDOA_TAG_TASK_STACK_SIZE 256

//Nodes IDS
#define MASTER_NODE_ID 0xc439

//TODO: Update these value to match respective slaves Addrs
#define SLAVE_NODE1_ID 0x8817
#define SLAVE_NODE2_ID 0x28BB
#define SLAVE_NODE3_ID 0xCBB3
#define SLAVE_NODE4_ID 0x5132
#define SLAVE_NODE5_ID 0x05
#define SLAVE_NODE6_ID 0x06


struct os_task rtdoa_tag_task_init;
os_stack_t rtdoa_tag_task_stack[RTDOA_TAG_TASK_STACK_SIZE];


struct tag_raging_packet {
    int distToNodeOne;
    int distToNodeTwo;
    int distToNodeThree;
    int distToNodeFour;
    int distToNodeFive;
    int distToNodeSix;
    int distToNodeSeven;
};

void rtdoa_tag_task(void *arg);

void rr_tag_packet(struct rtdoabh_tag_results_pkg *p);

#endif