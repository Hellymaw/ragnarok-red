#ifndef RR_TAG_H
#define RR_TAG_G

#define LED_TASK_PRIOR 10
#define LED_TASK_STACK_SIZE 250

#define RTDOA_TAG_TASK_PRIOR 5
#define RTDOA_TAG_TASK_STACK_SIZE 1024

//Nodes IDS
#define MASTER_NODE_ID 0xc439

struct os_task led_task_init, rtdoa_tag_task_init;
os_stack_t led_task_stack[LED_TASK_STACK_SIZE];
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

void blink_led_task(void *arg);

void rtdoa_tag_task(void *arg);

void rr_tag_packet(struct rtdoabh_tag_results_pkg *p);

#endif