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

#include "rr_node.h"

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
    int rc;

    sysinit();

    //Init blink led task
    os_task_init(&led_task_init, "blink_led_task", blink_led_task, NULL, LED_TASK_PRIOR,
                 OS_WAIT_FOREVER, led_task_stack, LED_TASK_STACK_SIZE);

    
    //Init RTDOA Node task
    os_task_init(&rtdoa_node_task_init, "rtdoa_node_task", rtdoa_node_task, NULL, RTDOA_NODE_TASK_PRIOR,
                 OS_WAIT_FOREVER, rtdoa_node_task_stack, RTDOA_NODE_TASK_STACK_SIZE);                 

    while (1) {
        /* Wait one second */
        os_time_delay(OS_TICKS_PER_SEC);
    }
    assert(0);
    return rc;
}
