#ifndef RR_TAG_H
#define RR_TAG_G

#include "os/os.h"
#include "rtdoa_backhaul/rtdoa_backhaul.h"

#define RTDOA_TAG_TASK_PRIOR 5
#define RTDOA_TAG_TASK_STACK_SIZE 256

//Nodes IDS
#define MASTER_NODE_ID 0xc439

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