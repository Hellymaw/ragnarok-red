/**
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 * 
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */
/**
 ************************************************************************
 * @file main.c
 * @author Wilfred MK
 * @date 19.05.2021 (Last Updated)
 * @brief Main entry this, initialises aux tasks. 
 **********************************************************************
 **/

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "sysinit/sysinit.h"
#include "os/os.h"
#include "bsp/bsp.h"
#include "imgmgr/imgmgr.h"
#include "hal/hal_gpio.h"
#include "hal/hal_bsp.h"
#include <hal/hal_system.h>
#ifdef ARCH_sim
#include "mcu/mcu_sim.h"
#endif

#include "rtdoa/rr_node.h"

#include "mesh/mesh.h"

#include "gpio/app_gpio.h"
#include "gpio/heartbeat_led.h"

#include "mesh/ble_mesh.h"
#include "mesh/device_composition.h"

#define SERIAL_OUTPUT_TASK_PRIORITY 7
#define SERIAL_OUTPUT_TASK_STACK_SIZE 128

#define SLAVE_NODE1_ID 0x8817
#define SLAVE_NODE2_ID 0x28BB
#define SLAVE_NODE3_ID 0xC6B3
#define SLAVE_NODE4_ID 0x5132
#define SLAVE_NODE5_ID 0x256A
#define SLAVE_NODE6_ID 0x19AB

struct os_task serial_output_task_task;
os_stack_t serial_output_task_stack[SERIAL_OUTPUT_TASK_STACK_SIZE];


uint16_t get_node_addr(uint8_t nodeIndex)
{
    switch (nodeIndex) {
    case 1:
        return SLAVE_NODE1_ID;
    case 2:
        return SLAVE_NODE2_ID;
    case 3:
        return SLAVE_NODE3_ID;
    case 4:
        return SLAVE_NODE4_ID;
    case 5:
        return SLAVE_NODE5_ID;
    case 6:
        return SLAVE_NODE6_ID;
    default:
        break;
    }
    return 0;
}


/**
 * @brief Task to output received blemesh data over UART
 * 
 * @param arg N/A
 */
void serial_output_task(void *arg)
{
    struct uwbData data = {0};

    while (1) {

        if (os_sem_pend(&receivedTagOne, os_time_ms_to_ticks32(100)) == OS_OK) {

            data = testingRangeT1;

            printf("%d,%x,%ld,-73,%x,%ld,-73,%x,%ld,-73,%x,%ld,-73,%x,%ld,-73\n", data.tagNum, get_node_addr(1), data.r1, get_node_addr(2), data.r2, get_node_addr(3), data.r3, get_node_addr(4), data.r4, get_node_addr(5), data.r5);
        }

        if (os_sem_pend(&receivedTagTwo, os_time_ms_to_ticks32(100)) == OS_OK) {

            data = testingRangeT2;

            printf("%d,%x,%ld,-73,%x,%ld,-73,%x,%ld,-73,%x,%ld,-73,%x,%ld,-73\n", data.tagNum, get_node_addr(1), data.r1, get_node_addr(2), data.r2, get_node_addr(3), data.r3, get_node_addr(4), data.r4, get_node_addr(5), data.r5);
        }
    }
}

/**
 * main
 *
 * The main task for the project. This function initializes packages,
 * and then blinks the BSP LED in a loop.
 *
 * @return int NOTE: this function should never return!
 */
int
main(int argc, char **argv)
{
    
    sysinit();

    app_gpio_init();

	init_pub();

    init_subscriber_sems();

    // Initialize the NimBLE host configuration.
	ble_hs_cfg.reset_cb = blemesh_on_reset;
	ble_hs_cfg.sync_cb = blemesh_on_sync;
	ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // init_timer();

    //Init blink led task
    os_task_init(&heartbeat_led_task_init, "heartbeat_led_task", 
            heartbeat_led_task, NULL, HEARTBEAT_LED_TASK_PRIORITY, 
            OS_WAIT_FOREVER, heartbeat_led_task_stack, 
            HEARTBEAT_LED_TASK_STACK_SIZE);

    os_task_init(&serial_output_task_task, "range_serial_output", 
            serial_output_task, NULL, SERIAL_OUTPUT_TASK_PRIORITY, 
            OS_WAIT_FOREVER, serial_output_task_stack, 
            SERIAL_OUTPUT_TASK_STACK_SIZE);

    //Init RTDOA Node task
    os_task_init(&rtdoa_node_task_init, "rtdoa_node_task", rtdoa_node_task, 
            NULL, RTDOA_NODE_TASK_PRIORITY, OS_WAIT_FOREVER, 
            rtdoa_node_task_stack, RTDOA_NODE_TASK_STACK_SIZE);                 

    while (1) {

        os_eventq_run(os_eventq_dflt_get());
    }
    
    return 0;
}
