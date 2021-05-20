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
#include "console/console.h"
#include "hal/hal_gpio.h"
#include "mesh/mesh.h"

#include "app_gpio.h"

#include "mesh/ble_mesh.h"
#include "mesh/device_composition.h"


int main(void)
{
	/* Initialize OS */
	sysinit();

	app_gpio_init();

	init_pub();

	printk("Initializing...\n");

	// Initialize the NimBLE host configuration.
	ble_hs_cfg.reset_cb = blemesh_on_reset;
	ble_hs_cfg.sync_cb = blemesh_on_sync;
	ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

	while (1) {
	
		os_eventq_run(os_eventq_dflt_get());
	}

	return 0;
}
