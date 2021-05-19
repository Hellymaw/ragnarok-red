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

#ifndef _APP_GPIO_H
#define _APP_GPIO_H

// Button debounce time
#define BUTTON_DEBOUNCE_DELAY_MS 250

// Switch, keeps track of state and callbacks
struct sw {
    uint8_t sw_num;
    uint8_t onoff_state;
    struct os_callout button_work;
    struct os_callout button_timer;
};

// Buttons used
extern int button_device[];

// LEDs used
extern const int led_device[];

// Initialises GPIO
void app_gpio_init(void);

#endif
