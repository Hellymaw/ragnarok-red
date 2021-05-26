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
/**
 ************************************************************************
 * @file ble_mesh.c
 * @author Aaron Helmore
 * @date 19.05.2021 (Last Updated)
 * @brief Contains BLE mesh callback functions, used in setting up a 
 * 			BLE Mesh
 * @note Used for a TAG node
 **********************************************************************
 **/


#include "console/console.h"

#include "ble_mesh.h"
#include "device_composition.h"

// Output OOB number
static int output_number(bt_mesh_output_action_t, u32_t);

// Output OOB string
static int output_string(const char *);

// Provisioning complete callback
static void prov_complete(u16_t net_idx, u16_t addr);

// Reset provisioning callback
static void prov_reset(void);

// Devices UUID
static u8_t dev_uuid[16] = MYNEWT_VAL(BLE_MESH_DEV_UUID);

// Device provisioning setup (aka UUID, OOB Auth, callbacks, etc.)
static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.output_size = 6,
	.output_actions = BT_MESH_DISPLAY_NUMBER | BT_MESH_DISPLAY_STRING,
	.output_number = output_number,
	.output_string = output_string,
	.complete = prov_complete,
	.reset = prov_reset,
};

/**
 * @brief Outputs number for OOB authentification
 */
static int output_number(bt_mesh_output_action_t action, u32_t number)
{
	printk("OOB Number: %lu\n", number);
	return 0;
}

/**
 * @brief Outputs string for OOB authentification
 */
static int output_string(const char *str)
{
	printk("OOB String: %s\n", str);
	return 0;
}

/**
 * @brief Called after blemesh provisioning is completed successfully
 * 
 * @param net_idx The network index
 * @param addr The primary network address
 */
static void prov_complete(u16_t net_idx, u16_t addr)
{

	printk("Local node provisioned, primary address 0x%04x\n", addr);
}

/**
 * @brief Called when the node is told to reset provisioning
 */
static void prov_reset(void)
{

	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

/**
 * @brief Called when host and controlled are reset due to error
 * 
 * @param reason reason for reset
 */
void blemesh_on_reset(int reason)
{

	BLE_HS_LOG(ERROR, "Resetting state; reason=%d\n", reason);
}

/**
 * @brief Called when host and controller sync after power on or reset
 */
void blemesh_on_sync(void)
{
	
	int err;
	ble_addr_t addr;

	console_printf("Bluetooth initialized\n");

	// Use Non-Resolvable Private Address (NRPA) 
	err = ble_hs_id_gen_rnd(1, &addr);
	assert(err == 0);
	err = ble_hs_id_set_rnd(addr.val);
	assert(err == 0);

	// Attempt to initialise mesh
	err = bt_mesh_init(addr.type, &prov, &composition);
	if (err) {
		
		console_printf("Initializing mesh failed (err %d)\n", err);
		return;
	}

	// If persistent blemesh config is enabled, attempt to load settings
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		
		console_printf("Attempting to restore mesh network from flash...\n");
		settings_load();
	}

	// Check to see if the mesh config was restored from flash
	if (bt_mesh_is_provisioned()) {

		console_printf("Mesh network restored from flash\n");
	} else {

		console_printf("Mesh network was unable to be restored from flash\n");
	}

	// Enable provisioning
	bt_mesh_prov_enable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);

	console_printf("Mesh initialized\n");
}
