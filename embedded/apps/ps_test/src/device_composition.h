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

#ifndef _DEVICE_COMPOSITION_H
#define _DEVICE_COMPOSITION_H

#define CID_RUNTIME 0x05C3

#define STATE_OFF	0x00
#define STATE_ON	0x01
#define STATE_DEFAULT	0x01
#define STATE_RESTORE	0x02


struct onoff_state {
    uint8_t current;
    uint8_t previous;
    uint8_t led_gpio_pin;
};

struct vendor_state {
	int current;
	uint32_t response;
	uint8_t last_tid;
	uint16_t last_src_addr;
	uint16_t last_dst_addr;
	int64_t last_msg_timestamp;
};

extern struct generic_onoff_state gen_onoff_srv_root_user_data;

extern struct bt_mesh_model root_models[];
extern struct bt_mesh_model vnd_models[];

extern const struct bt_mesh_comp comp;

extern struct bt_mesh_model *mod_cli_sw[];
extern struct bt_mesh_model *mod_srv_sw[];

void gen_onoff_set_unack(struct bt_mesh_model *model,
                                struct bt_mesh_msg_ctx *ctx,
                                struct os_mbuf *buf);

#endif
