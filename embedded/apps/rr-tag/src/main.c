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
#include "rr_tag.h"
#ifdef ARCH_sim
#include "mcu/mcu_sim.h"
#endif

#include "console/console.h"
#include "mesh/mesh.h"

#include "app_gpio.h"
#include "heartbeat_led.h"

#include "mesh/ble_mesh.h"
#include "mesh/device_composition.h"




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

    app_gpio_init();

	init_pub();

    ble_hs_cfg.reset_cb = blemesh_on_reset;
	ble_hs_cfg.sync_cb = blemesh_on_sync;
	ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    //Init blink led task
    os_task_init(&heartbeat_led_task_init, "heartbeat_led_task", heartbeat_led_task, NULL, HEART_LED_TASK_PRIORITY,
                 OS_WAIT_FOREVER, heartbeat_led_task_stack, HEART_LED_TASK_STACK_SIZE);

    //Init RTDOA tag task
    os_task_init(&rtdoa_tag_task_init, "rtdoa_tag_task", rtdoa_tag_task, NULL, RTDOA_TAG_TASK_PRIOR,
                 OS_WAIT_FOREVER, rtdoa_tag_task_stack, RTDOA_TAG_TASK_STACK_SIZE);

  
    while (1) {
        /* Wait one second */
        // os_time_delay(OS_TICKS_PER_SEC);
        os_eventq_run(os_eventq_dflt_get());
    }
    assert(0);
    return rc;
}


