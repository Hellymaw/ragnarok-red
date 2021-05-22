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

#ifndef _BLE_MESH_H
#define _BLE_MESH_H

#include "mesh/mesh.h"
#include "mesh/glue.h"

// Enable Out Of Band authentification
#define OOB_AUTH_ENABLE 1

// NOTE: These are removed as `device_composition.c` uses BT_MESH_MODEL_OP_2 directly
// Model Operation Codes for Generic ONOFF
#define BT_MESH_MODEL_OP_GEN_ONOFF_GET		BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET		BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GEN_ONOFF_STATUS	BT_MESH_MODEL_OP_2(0x82, 0x04)

// Primary address of network (aka not subnet)
extern uint16_t primary_addr;

// Network index
extern uint16_t primary_net_idx;

// Callback for when host and controller reset due to error
void blemesh_on_reset(int reason);

// Callback for when host and controller sync after power on or reset (not due to error)
void blemesh_on_sync(void);

// NOTE: This is actually defined in 'device_composition.c', unsure if this can move
// Initialise publications
void init_pub(void);

#endif

