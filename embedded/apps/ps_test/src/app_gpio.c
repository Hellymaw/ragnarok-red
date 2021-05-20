/*
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

/* Bluetooth: Mesh Generic OnOff, Generic Level, Lighting & Vendor Models
 *
 * Copyright (c) 2018 Vikrant More
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// #include "console/console.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
// #include "mesh/mesh.h"

#include "app_gpio.h"

#include "mesh/ble_mesh.h"
#include "mesh/device_composition.h"

// // Buttons used
// int button_device[] = {
// 	BUTTON_1,
// };

// // LEDs used 
// const int led_device[] = {
// 	LED_1,
// 	LED_2,
// 	LED_3,
// 	LED_4,
// };

// Switch
// static struct sw sw;

// // Last & currect button press time
// static uint32_t time, last_time;

// // Number of button presses
// static uint8_t button_press_cnt;

// // NOTE: This should be removed during actual mesh implementation
// // Count of transistion??
// static uint8_t trans_id;

// // Button event (this is an event that is placed into event queue by an IRQ)
// static struct os_event button_event;

/**
 * @brief Map GPIO pins to button number
 * 
 * @param pin_pos Position of pin
 * @return uint8_t Index of pin for sw
 */
// static uint8_t pin_to_sw(int pin_pos)
// {
//     switch (pin_pos) {
//         case BUTTON_1: return 0;
//         default:break;
//     }

//     BT_ERR("No match for GPIO pin 0x%08x", pin_pos);
//     return 0;
// }

/**
 * @brief Handles the button_pressed event
 *        Also debounces button press
 * 
 * @param ev button_pressed event
 */
// static void button_pressed(struct os_event *ev)
// {
//     int pin_pos = (int ) ev->ev_arg;
//     /*
//      * One button press within a 1 second interval sends an on message
//      * More than one button press sends an off message
//      */

//     time = k_uptime_get_32();

//     /* debounce the switch */
//     if (time < last_time + BUTTON_DEBOUNCE_DELAY_MS) {
//         last_time = time;
//         return;
//     }

//     if (button_press_cnt == 0) {
//         os_callout_reset(&sw.button_timer, os_time_ms_to_ticks32(K_SECONDS(1)));
//     }

//     BT_INFO("button_press_cnt 0x%02x", button_press_cnt);
//     button_press_cnt++;

//     /* The variable pin_pos is the pin position in the GPIO register,
//      * not the pin number. It's assumed that only one bit is set.
//      */

//     sw.sw_num = pin_to_sw(pin_pos);
//     last_time = time;
// }

// NOTE: Unsure why this is called 'Timer'
/*
 * Button Count Timer Worker
 */
// static void button_cnt_timer(struct os_event *work)
// {
//     struct sw *button_sw = work->ev_arg;

//     button_sw->onoff_state = button_press_cnt == 1 ? 1 : 0;
//     BT_INFO("button_press_cnt 0x%02x onoff_state 0x%02x",
//                 button_press_cnt, button_sw->onoff_state);
//     button_press_cnt = 0;
//     os_callout_reset(&sw.button_work, 0);
// }

// NOTE: we won't have buttons but note usage of 'goto'
/*
 * Button Pressed Worker Task
 */
// static void button_pressed_worker(struct os_event *work)
// {
//     struct os_mbuf *msg = NET_BUF_SIMPLE(1);
//     struct bt_mesh_model *mod_cli, *mod_srv;
//     struct bt_mesh_model_pub *pub_cli, *pub_srv;
//     struct sw *sw = work->ev_arg;
//     u8_t sw_idx = sw->sw_num;
//     int err;

//     mod_cli = mod_cli_sw[sw_idx];
//     pub_cli = mod_cli->pub;

//     mod_srv = led_onoff_server[sw_idx];
//     pub_srv = led_onoff_server->pub;
//     (void)pub_srv;

//     /* If unprovisioned, just call the set function.
//      * The intent is to have switch-like behavior
//      * prior to provisioning. Once provisioned,
//      * the button and its corresponding led are no longer
//      * associated and act independently. So, if a button is to
//      * control its associated led after provisioning, the button
//      * must be configured to either publish to the led's unicast
//      * address or a group to which the led is subscribed.
//      */

//     if (primary_addr == BT_MESH_ADDR_UNASSIGNED) {
//         struct bt_mesh_msg_ctx ctx = {
//                 .addr = sw_idx + primary_addr,
//         };

//         /* This is a dummy message sufficient
// 	 * for the led server
// 	 */

//         net_buf_simple_add_u8(msg, sw->onoff_state);
//         gen_onoff_set_unack(mod_srv, &ctx, msg);
//         goto done;
//     }

//     if (pub_cli->addr == BT_MESH_ADDR_UNASSIGNED) {
//         goto done;
//     }

//     BT_INFO("publish to 0x%04x onoff 0x%04x sw_idx 0x%04x",
//                 pub_cli->addr, sw->onoff_state, sw_idx);
//     bt_mesh_model_msg_init(pub_cli->msg,
//                            BT_MESH_MODEL_OP_GEN_ONOFF_SET);
//     net_buf_simple_add_u8(pub_cli->msg, sw->onoff_state);
//     net_buf_simple_add_u8(pub_cli->msg, trans_id++);
//     err = bt_mesh_model_publish(mod_cli);
//     if (err) {
//         BT_ERR("bt_mesh_model_publish err %d", err);
//     }

// done:
//     os_mbuf_free_chain(msg);
// }

/**
 * @brief Interrupt handler for GPIO, adds 'button_event' to event queue
 */
// static void gpio_irq_handler(void *arg)
// {
// 	button_event.ev_arg = arg;
// 	os_eventq_put(os_eventq_dflt_get(), &button_event);
// }

/**
 * @brief Initialises GPIO
 */
void app_gpio_init(void)
{
	/* LEDs configiuratin & setting */

	hal_gpio_init_out(LED_1, 1);
	// hal_gpio_init_out(led_device[1], 1);
	// hal_gpio_init_out(led_device[2], 1);
	// hal_gpio_init_out(led_device[3], 1);

	/* Buttons configiuratin & setting */
	/* Initialize button worker task*/
    // os_callout_init(&sw.button_work, os_eventq_dflt_get(),
    //                 button_pressed_worker, &sw);

    // /* Initialize button count timer */
    // os_callout_init(&sw.button_timer, os_eventq_dflt_get(),
    //                 button_cnt_timer, &sw);
	// button_event.ev_cb = button_pressed;

	// hal_gpio_irq_init(button_device[0], gpio_irq_handler, NULL,
	// 		  HAL_GPIO_TRIG_FALLING, HAL_GPIO_PULL_UP);
	// hal_gpio_irq_enable(button_device[0]);

}

