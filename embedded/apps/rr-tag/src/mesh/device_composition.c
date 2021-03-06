/**
 ************************************************************************
 * @file device_composition.c
 * @author Aaron Helmore
 * @date 24.05.2021 (Last Updated)
 * @brief Contains BLE mesh device composition configuration code. 
 * @note Used for an tag node
 **********************************************************************
 **/

#include "device_composition.h"

#include "console/console.h"
#include "hal/hal_gpio.h"
#include "bsp/bsp.h"
#include "mesh/mesh.h"

#include "ble_mesh.h"

// Ranges for slave 1 & 2
struct tag_ranges ranges1 = {
    .r1 = 0x1001,
    .r2 = 0x2000
};

// Ranges for slave 3 & 4
struct tag_ranges ranges2 = {
    .r1 = 0x3001,
    .r2 = 0x4000
};

// Ranges for slave 5 & 6
struct tag_ranges ranges3 = {
    .r1 = 0x5001,
    .r2 = 0x6000
};

// Have the TIDs been randomised
static bool is_randomisation_of_TIDs_done;

// TID for vnd_range publishing
static uint8_t tid_vnd_range;

// Pins for LEDs used by onoff model
static struct onoff_state onoff_server_led = {
    .led_gpio_pin = LED_1
};

// Configuration server
static struct bt_mesh_cfg_srv config_server = {
	.relay = BT_MESH_RELAY_ENABLED,
	.beacon = BT_MESH_BEACON_ENABLED,
	.gatt_proxy = BT_MESH_GATT_PROXY_ENABLED,
	.default_ttl = 7,
	/* 2 transmissions with 20ms interval */
	.net_transmit = BT_MESH_TRANSMIT(2, 20),
	/* 3 transmissions with 20ms interval */
	.relay_retransmit = BT_MESH_TRANSMIT(3, 20),
};

// Device mesh health server
static struct bt_mesh_health_srv health_server = {};

// Device elements
static struct bt_mesh_elem elements[];

// Publishers
static struct bt_mesh_model_pub health_pub;
static struct bt_mesh_model_pub gen_onoff_server_pub;
static struct bt_mesh_model_pub vnd_range_server_pub_1;
static struct bt_mesh_model_pub vnd_range_server_pub_2;
static struct bt_mesh_model_pub vnd_range_server_pub_3;

// Buffers for publishers
static struct os_mbuf *bt_mesh_pub_msg_health_pub;
static struct os_mbuf *bt_mesh_pub_msg_gen_onoff_server_pub;
static struct os_mbuf *bt_mesh_pub_msg_vnd_range_server_pub_1;
static struct os_mbuf *bt_mesh_pub_msg_vnd_range_server_pub_2;
static struct os_mbuf *bt_mesh_pub_msg_vnd_range_server_pub_3;

/**
 * @brief Initialise and configure publisher buffers
 */
void init_pub(void)
{
    
    // Initialise buffers
    bt_mesh_pub_msg_health_pub = NET_BUF_SIMPLE(1 + 3 + 0);
    bt_mesh_pub_msg_gen_onoff_server_pub = NET_BUF_SIMPLE(2 + 2);
    bt_mesh_pub_msg_vnd_range_server_pub_1 = NET_BUF_SIMPLE(3 + 4 + 4);
    bt_mesh_pub_msg_vnd_range_server_pub_2 = NET_BUF_SIMPLE(3 + 4 + 4);
    bt_mesh_pub_msg_vnd_range_server_pub_3 = NET_BUF_SIMPLE(3 + 4 + 4);

    // Configure buffers
    health_pub.msg = bt_mesh_pub_msg_health_pub;
    gen_onoff_server_pub.msg = bt_mesh_pub_msg_gen_onoff_server_pub;
    vnd_range_server_pub_1.msg = bt_mesh_pub_msg_vnd_range_server_pub_1;
    vnd_range_server_pub_2.msg = bt_mesh_pub_msg_vnd_range_server_pub_2;
    vnd_range_server_pub_3.msg = bt_mesh_pub_msg_vnd_range_server_pub_3;
}


/**
 * @brief Randomise the TIDs for publishers
 */
void randomise_publishers_TID(void)
{

	bt_rand(&tid_vnd_range, sizeof(tid_vnd_range));

	is_randomisation_of_TIDs_done = true;
}

/**
 * @brief Publishes the state of a vnd_range model
 * 
 * @param model Publishing model
 */
void vnd_range_publish(struct bt_mesh_model *model)
{
    int err;
    struct os_mbuf *msg = model->pub->msg;
    struct tag_ranges *ranges = model->user_data;

    if (model->pub->addr == BT_MESH_ADDR_UNASSIGNED) {

        return;
    }

    // Initialise status message
    bt_mesh_model_msg_init(msg, VND_RANGE_MODEL_STATUS_OPCODE);
    net_buf_simple_add_le32(msg, ranges->r1);
    net_buf_simple_add_le32(msg, ranges->r2);

    // Attempt to publish status message
    err = bt_mesh_model_publish(model);
    if (err) {

        printk("bt_mesh_model_publish err %d\n", err);
    }
}

/**
 * @brief Generic onoff model 'get' message handler
 * 
 * @param model Calling mesh model
 * @param ctx Message context
 * @param buf Message data
 */
static void gen_onoff_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx 
        *ctx, struct os_mbuf *buf)
{
    struct os_mbuf *msg = NET_BUF_SIMPLE(2 + 1 + 4);
    struct onoff_state *state = model->user_data;

    // Log received message
    BT_INFO("addr 0x%04x onoff 0x%02x", bt_mesh_model_elem(model)->addr, state->current);

    // Initialise status message
    bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_2(0x82, 0x04));
    net_buf_simple_add_u8(msg, state->current);

    if (bt_mesh_model_send(model, ctx, msg, NULL, NULL)) {

        BT_ERR("Unable to send onoff status response");
    }

    os_mbuf_free_chain(msg);
}

/**
 * @brief Generic onoff model set unack message handler
 * 
 * @param model Calling mesh model
 * @param ctx Message context
 * @param buf Message data
 */
static void gen_onoff_set_unack(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct os_mbuf *buf)
{
    struct os_mbuf *msg = model->pub->msg;
    struct onoff_state *state = model->user_data;
    int err;

    state->current = net_buf_simple_pull_u8(buf);

    // Log current state
    BT_INFO("addr 0x%02x state 0x%02x");

    hal_gpio_write(state->led_gpio_pin, (state->current) ? 0 : 1);

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
        BT_INFO("publish last 0x%02x cur 0x%02x", state->previous, state->current);
        
        state->previous = state->current;

        bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_2(0x82, 0x04));
        net_buf_simple_add_u8(msg, state->current);
        
        err = bt_mesh_model_publish(model);
        if (err) {
        
            BT_ERR("bt_mesh_model_publish err %d", err);
        }
    }
}

/**
 * @brief Generic onoff server set message handler
 * 
 * @param model Calling mesh model
 * @param ctx Message context
 * @param buf Message buffer
 */
static void gen_onoff_set(struct bt_mesh_model *model, 
        struct bt_mesh_msg_ctx *ctx, struct os_mbuf *buf)
{
    BT_INFO("");

    gen_onoff_set_unack(model, ctx, buf);
    gen_onoff_get(model, ctx, buf);
}

/**
 * @brief Vendor range infor server get message handler
 * 
 * @param model Calling mesh model
 * @param ctx Message context
 * @param buf Message buffer
 */
static void vnd_range_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct os_mbuf *buf)
{
    struct os_mbuf *msg = NET_BUF_SIMPLE(3 + 8 + 4);
    struct tag_ranges *ranges = model->user_data;

    // Log received message
    BT_INFO("addr 0x%04x vnd_range r1:%d, r2:%d", bt_mesh_model_elem(model)->addr, ranges->r1, ranges->r2);

    // Initialise status message
    bt_mesh_model_msg_init(msg, VND_RANGE_MODEL_STATUS_OPCODE);
    net_buf_simple_add_le32(msg, ranges->r1);
    net_buf_simple_add_le32(msg, ranges->r2);

    if (bt_mesh_model_send(model, ctx, msg, NULL, NULL)) {

        BT_ERR("Unable to send onoff status response");
    }

    os_mbuf_free_chain(msg);
}

// Opcodes for Generic OnOff Server model
static const struct bt_mesh_model_op gen_onoff_server_opcodes[] = {
    {BT_MESH_MODEL_OP_2(0x82, 0x01), 0, gen_onoff_get},
    {BT_MESH_MODEL_OP_2(0x82, 0x02), 2, gen_onoff_set},
    {BT_MESH_MODEL_OP_2(0x82, 0x03), 2, gen_onoff_set_unack},
    BT_MESH_MODEL_OP_END
};

// Opcodes for Vendor Range Server model
static const struct bt_mesh_model_op vnd_range_server_opcodes[] = {
    {VND_RANGE_MODEL_GET_OPCODE, 0, vnd_range_get},
    BT_MESH_MODEL_OP_END
};

// Setup of device root element models
struct bt_mesh_model root_models[] = {
    BT_MESH_MODEL_CFG_SRV(&config_server),
    BT_MESH_MODEL_HEALTH_SRV(&health_server, &health_pub),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_server_opcodes, &gen_onoff_server_pub, &onoff_server_led),
};

// Vendor Models for element 2
struct bt_mesh_model vnd_models[] = {
    BT_MESH_MODEL_VND(CID_RUNTIME, 0x4001, vnd_range_server_opcodes, &vnd_range_server_pub_1, &ranges1),
    BT_MESH_MODEL_VND(CID_RUNTIME, 0x4002, vnd_range_server_opcodes, &vnd_range_server_pub_2, &ranges2),
    BT_MESH_MODEL_VND(CID_RUNTIME, 0x4003, vnd_range_server_opcodes, &vnd_range_server_pub_3, &ranges3),
};

// Setup LED OnOff server
struct bt_mesh_model *led_onoff_server = &root_models[2];

// Setup of device elements
static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
    BT_MESH_ELEM(0, BT_MESH_MODEL_NONE, vnd_models)
};

// Device mesh composition
const struct bt_mesh_comp composition = {
    .cid = CID_RUNTIME,
    .elem = elements, 
    .elem_count = ARRAY_SIZE(elements)
};