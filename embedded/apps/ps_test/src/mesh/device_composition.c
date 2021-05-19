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
#include "bsp/bsp.h"
#include "mesh/mesh.h"

#include "app_gpio.h"
#include "ble_mesh.h"
#include "device_composition.h"

// Pins for LEDs used by onoff model
static struct onoff_state onoff_state_arr[] = {
        { .led_gpio_pin = LED_1 },
        { .led_gpio_pin = LED_2 },
        { .led_gpio_pin = LED_3 },
        { .led_gpio_pin = LED_4 },
};

// Configuration server
static struct bt_mesh_cfg_srv cfg_srv = {
	.relay = BT_MESH_RELAY_ENABLED,
	.beacon = BT_MESH_BEACON_ENABLED,

#if defined(CONFIG_BT_MESH_FRIEND)
	.frnd = BT_MESH_FRIEND_ENABLED,
#else
	.frnd = BT_MESH_FRIEND_NOT_SUPPORTED,
#endif

#if defined(CONFIG_BT_MESH_GATT_PROXY)
	.gatt_proxy = BT_MESH_GATT_PROXY_ENABLED,
#else
	.gatt_proxy = BT_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif

	.default_ttl = 7,

	/* 2 transmissions with 20ms interval */
	.net_transmit = BT_MESH_TRANSMIT(2, 20),

	/* 3 transmissions with 20ms interval */
	.relay_retransmit = BT_MESH_TRANSMIT(3, 20),
};

/*
 * Client Configuration Declaration
 */

// Configuration client
static struct bt_mesh_cfg_cli cfg_cli = {};

// Device health server
static struct bt_mesh_health_srv health_srv = {};

// TODO: Double check these descriptions are correct

// Publishers
static struct bt_mesh_model_pub health_pub;
static struct bt_mesh_model_pub gen_onoff_srv_pub_root;
static struct bt_mesh_model_pub gen_onoff_cli_pub_root;
static struct bt_mesh_model_pub vnd_pub;

// Buffers for publishers
static struct os_mbuf *bt_mesh_pub_msg_health_pub;
static struct os_mbuf *bt_mesh_pub_msg_gen_onoff_srv_pub_root;
static struct os_mbuf *bt_mesh_pub_msg_gen_onoff_cli_pub_root;
static struct os_mbuf *bt_mesh_pub_msg_vnd_pub;

/**
 * @brief Initialise publishers
 */
void init_pub(void)
{
	// Configure buffers
	bt_mesh_pub_msg_health_pub					= NET_BUF_SIMPLE(1 + 3 + 0);
	bt_mesh_pub_msg_gen_onoff_srv_pub_root		= NET_BUF_SIMPLE(2 + 2);
	bt_mesh_pub_msg_gen_onoff_cli_pub_root		= NET_BUF_SIMPLE(2 + 2);
	bt_mesh_pub_msg_vnd_pub						= NET_BUF_SIMPLE(3 + 6);
	
	// Configure publisher messages
	health_pub.msg					= bt_mesh_pub_msg_health_pub;
	gen_onoff_srv_pub_root.msg		= bt_mesh_pub_msg_gen_onoff_srv_pub_root;
	gen_onoff_cli_pub_root.msg		= bt_mesh_pub_msg_gen_onoff_cli_pub_root;
	vnd_pub.msg						= bt_mesh_pub_msg_vnd_pub;	
}

// Stores vencer state information
struct vendor_state vnd_user_data;

// Device elements
static struct bt_mesh_elem elements[];



/**
 * @brief Message handler for onoff server model get cmd
 */
static void gen_onoff_get(struct bt_mesh_model *model,
                          struct bt_mesh_msg_ctx *ctx,
                          struct os_mbuf *buf)
{
    struct os_mbuf *msg = NET_BUF_SIMPLE(2 + 1 + 4);
    struct onoff_state *state = model->user_data;

    BT_INFO("addr 0x%04x onoff 0x%02x",
                bt_mesh_model_elem(model)->addr, state->current);
    bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
    net_buf_simple_add_u8(msg, state->current);

    if (bt_mesh_model_send(model, ctx, msg, NULL, NULL)) {
        BT_ERR("Unable to send On Off Status response");
    }

    os_mbuf_free_chain(msg);
}

/**
 * @brief Message handler for onoff model unack
 */
void gen_onoff_set_unack(struct bt_mesh_model *model,
                                struct bt_mesh_msg_ctx *ctx,
                                struct os_mbuf *buf)
{
    struct os_mbuf *msg = model->pub->msg;
    struct onoff_state *state = model->user_data;
    int err;

    state->current = net_buf_simple_pull_u8(buf);
    BT_INFO("addr 0x%02x state 0x%02x",
                bt_mesh_model_elem(model)->addr, state->current);

    /* Pin set low turns on LED's on the nrf52840-pca10056 board */
    hal_gpio_write(state->led_gpio_pin, state->current ? 0 : 1);

    /*
     * If a server has a publish address, it is required to
     * publish status on a state change
     *
     * See Mesh Profile Specification 3.7.6.1.2
     *
     * Only publish if there is an assigned address
     */

    if (state->previous != state->current &&
        model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
        BT_INFO("publish last 0x%02x cur 0x%02x",
                    state->previous,
                    state->current);
        state->previous = state->current;
        bt_mesh_model_msg_init(msg,
                               BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
        net_buf_simple_add_u8(msg, state->current);
        err = bt_mesh_model_publish(model);
        if (err) {
            BT_ERR("bt_mesh_model_publish err %d", err);
        }
    }
}

/**
 * @brief Message handler for onoff server model set cmd
 */
static void gen_onoff_set(struct bt_mesh_model *model,
                          struct bt_mesh_msg_ctx *ctx,
                          struct os_mbuf *buf)
{
    BT_INFO("");

    gen_onoff_set_unack(model, ctx, buf);
    gen_onoff_get(model, ctx, buf);
}

/**
 * @brief Message handler for onoff client model status cmd
 */
static void gen_onoff_status(struct bt_mesh_model *model,
                             struct bt_mesh_msg_ctx *ctx,
                             struct os_mbuf *buf)
{
    uint8_t	state;

    state = net_buf_simple_pull_u8(buf);

    BT_INFO("Node 0x%04x OnOff status from 0x%04x with state 0x%02x",
                bt_mesh_model_elem(model)->addr, ctx->addr, state);
}

/**
 * @brief Message handler for vendor model get cmd
 */
static void vnd_get(struct bt_mesh_model *model,
		    struct bt_mesh_msg_ctx *ctx,
		    struct os_mbuf *buf)
{
	struct os_mbuf *msg = NET_BUF_SIMPLE(3 + 6 + 4);
	struct vendor_state *state = model->user_data;

	/* This is dummy response for demo purpose */
	state->response = 0xA578FEB3;

	bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_3(0x04, CID_RUNTIME));
	net_buf_simple_add_le16(msg, state->current);
	net_buf_simple_add_le32(msg, state->response);

	if (bt_mesh_model_send(model, ctx, msg, NULL, NULL)) {
		printk("Unable to send VENDOR Status response\n");
	}

	os_mbuf_free_chain(msg);
}

/**
 * @brief Message handler for vendor model unack
 */
static void vnd_set_unack(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct os_mbuf *buf)
{
	u8_t tid;
	int current;
	s64_t now;
	struct vendor_state *state = model->user_data;

	current = net_buf_simple_pull_le16(buf);
	tid = net_buf_simple_pull_u8(buf);

	now = k_uptime_get();
	if (state->last_tid == tid &&
	    state->last_src_addr == ctx->addr &&
	    state->last_dst_addr == ctx->recv_dst &&
	    (now - state->last_msg_timestamp <= K_SECONDS(6))) {
		return;
	}

	state->last_tid = tid;
	state->last_src_addr = ctx->addr;
	state->last_dst_addr = ctx->recv_dst;
	state->last_msg_timestamp = now;
	state->current = current;

	printk("Vendor model message = %04x\n", state->current);

	if (state->current == STATE_ON) {
		/* LED2 On */
		hal_gpio_write(led_device[1], 0);
	} else {
		/* LED2 Off */
		hal_gpio_write(led_device[1], 1);
	}
}

/**
 * @brief Message handler for vendor model set cmd
 */
static void vnd_set(struct bt_mesh_model *model,
		    struct bt_mesh_msg_ctx *ctx,
		    struct os_mbuf *buf)
{
	vnd_set_unack(model, ctx, buf);
	vnd_get(model, ctx, buf);
}

/**
 * @brief Message handler for vendor model status cmd
 */
static void vnd_status(struct bt_mesh_model *model,
		       struct bt_mesh_msg_ctx *ctx,
		       struct os_mbuf *buf)
{
	printk("Acknownledgement from Vendor\n");
	printk("cmd = %04x\n", net_buf_simple_pull_le16(buf));
	printk("response = %08lx\n", net_buf_simple_pull_le32(buf));
}

/* Mapping of message handlers for Generic OnOff Server (0x1000) */
static const struct bt_mesh_model_op gen_onoff_srv_op[] = {
	{ BT_MESH_MODEL_OP_2(0x82, 0x01), 0, gen_onoff_get },
	{ BT_MESH_MODEL_OP_2(0x82, 0x02), 2, gen_onoff_set },
	{ BT_MESH_MODEL_OP_2(0x82, 0x03), 2, gen_onoff_set_unack },
	BT_MESH_MODEL_OP_END,
};

/* Mapping of message handlers for Generic OnOff Client (0x1001) */
static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
	{ BT_MESH_MODEL_OP_2(0x82, 0x04), 1, gen_onoff_status },
	BT_MESH_MODEL_OP_END,
};


/* Mapping of message handlers for Vendor (0x4321) */
static const struct bt_mesh_model_op vnd_ops[] = {
	{ BT_MESH_MODEL_OP_3(0x01, CID_RUNTIME), 0, vnd_get },
	{ BT_MESH_MODEL_OP_3(0x02, CID_RUNTIME), 3, vnd_set },
	{ BT_MESH_MODEL_OP_3(0x03, CID_RUNTIME), 3, vnd_set_unack },
	{ BT_MESH_MODEL_OP_3(0x04, CID_RUNTIME), 6, vnd_status },
	BT_MESH_MODEL_OP_END,
};

// Setup of device root element models
struct bt_mesh_model root_models[] = {
	BT_MESH_MODEL_CFG_SRV(&cfg_srv),
	BT_MESH_MODEL_CFG_CLI(&cfg_cli),
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV,
		      gen_onoff_srv_op, &gen_onoff_srv_pub_root,
		      &onoff_state_arr[0]),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI,
		      gen_onoff_cli_op, &gen_onoff_cli_pub_root,
		      &onoff_state_arr[0]),
};

// Setup of device vendor models
struct bt_mesh_model vnd_models[] = {
	BT_MESH_MODEL_VND(CID_RUNTIME, 0x4321, vnd_ops,
			  &vnd_pub, &vnd_user_data),
};

// Setup of device elements
static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models,  vnd_models),
};

// Setup of device composition
const struct bt_mesh_comp comp = {
	.cid = CID_RUNTIME,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

/*
 * Button to Client Model Assignments
 */
struct bt_mesh_model *mod_cli_sw[] = {
        &root_models[4],
};

/*
 * LED to Server Model Assigmnents
 */
struct bt_mesh_model *mod_srv_sw[] = {
        &root_models[3],
};