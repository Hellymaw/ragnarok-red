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

#include "rtdoa/rr_tag.h"

#include "console/console.h"
#include "mesh/mesh.h"

#include "gpio/app_gpio.h"
#include "gpio/heartbeat_led.h"

#include "mesh/ble_mesh.h"
#include "mesh/device_composition.h"

// OS timer to publish tag range information
struct os_callout tag_publish_timer;

/**
 * @brief Handler for tag publishing timer
 * 
 * @param dummy 
 */
static void tag_publish_timer_handler(struct os_event *dummy)
{
    // Publish all ranging information and reset timer
    vnd_range_publish(&vnd_models[0]);
    vnd_range_publish(&vnd_models[1]);
    vnd_range_publish(&vnd_models[2]);

    os_callout_reset(&tag_publish_timer, os_time_ms_to_ticks32(250));
}

/**
 * @brief Initialise publishing timer
 * 
 */
static void init_timer(void)
{

    // Initialise timer
    os_callout_init(&tag_publish_timer, os_eventq_dflt_get(), 
            tag_publish_timer_handler, NULL);

    // Start timer
    os_callout_reset(&tag_publish_timer, os_time_ms_to_ticks32(250));
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
    
    // Initialise OS
    sysinit();

    // Intialise GPIO
    app_gpio_init();

    // Initialise publisher buffers
	init_pub();

    // Setup BLE mesh callbacks
    ble_hs_cfg.reset_cb = blemesh_on_reset;
	ble_hs_cfg.sync_cb = blemesh_on_sync;
	ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // Initialise publisher timer
    init_timer();

    //Init blink led task
    os_task_init(&heartbeat_led_task_init, "heartbeat_led_task", 
            heartbeat_led_task, NULL, HEARTBEAT_LED_TASK_PRIORITY, 
            OS_WAIT_FOREVER, heartbeat_led_task_stack, 
            HEARTBEAT_LED_TASK_STACK_SIZE);

    //Init RTDOA tag task
    os_task_init(&rtdoa_tag_task_init, "rtdoa_tag_task", rtdoa_tag_task, NULL, 
            RTDOA_TAG_TASK_PRIOR, OS_WAIT_FOREVER, rtdoa_tag_task_stack, 
            RTDOA_TAG_TASK_STACK_SIZE);

  
    while (1) {
        // Get top of OS event queue and run
        os_eventq_run(os_eventq_dflt_get());
    }
    
    return 0;
}


