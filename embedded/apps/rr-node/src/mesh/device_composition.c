#include "device_composition.h"

#include "console/console.h"
#include "hal/hal_gpio.h"
#include "bsp/bsp.h"
#include <os/os.h>
#include "mesh/mesh.h"

#include "ble_mesh.h"

struct os_sem receivedTagOne;
struct os_sem receivedTagTwo;

struct uwbData testingRangeT1 = {
    .tagNum = 1,
};
struct uwbData testingRangeT2 = {
    .tagNum = 2,
};

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

static struct bt_mesh_elem elements[];

// Publishers
static struct bt_mesh_model_pub health_pub;
static struct bt_mesh_model_pub gen_onoff_server_pub;

static struct bt_mesh_model_pub vnd_range_client_pub_1_1;
static struct bt_mesh_model_pub vnd_range_client_pub_1_2;
static struct bt_mesh_model_pub vnd_range_client_pub_1_3;

static struct bt_mesh_model_pub vnd_range_client_pub_2_1;
static struct bt_mesh_model_pub vnd_range_client_pub_2_2;
static struct bt_mesh_model_pub vnd_range_client_pub_2_3;

static struct os_mbuf *bt_mesh_pub_msg_health_pub;
static struct os_mbuf *bt_mesh_pub_msg_gen_onoff_server_pub;
static struct os_mbuf *bt_mesh_pub_msh_vnd_range_client_pub_1_1;
static struct os_mbuf *bt_mesh_pub_msh_vnd_range_client_pub_1_2;
static struct os_mbuf *bt_mesh_pub_msh_vnd_range_client_pub_1_3;

static struct os_mbuf *bt_mesh_pub_msh_vnd_range_client_pub_2_1;
static struct os_mbuf *bt_mesh_pub_msh_vnd_range_client_pub_2_2;
static struct os_mbuf *bt_mesh_pub_msh_vnd_range_client_pub_2_3;

/**
 * @brief Initialise and configure publisher buffers
 */
void init_pub(void)
{
    // TODO: figure out what sizes mean
    // TODO: Change sizes
    // Initialise buffers
    bt_mesh_pub_msg_health_pub = NET_BUF_SIMPLE(1 + 3 + 0);
    bt_mesh_pub_msg_gen_onoff_server_pub = NET_BUF_SIMPLE(2 + 2);

    bt_mesh_pub_msh_vnd_range_client_pub_1_1 = NET_BUF_SIMPLE(3 + 4 + 4);
    bt_mesh_pub_msh_vnd_range_client_pub_1_2 = NET_BUF_SIMPLE(3 + 4 + 4);
    bt_mesh_pub_msh_vnd_range_client_pub_1_3 = NET_BUF_SIMPLE(3 + 4 + 4);
    
    bt_mesh_pub_msh_vnd_range_client_pub_2_1 = NET_BUF_SIMPLE(3 + 4 + 4);
    bt_mesh_pub_msh_vnd_range_client_pub_2_2 = NET_BUF_SIMPLE(3 + 4 + 4);
    bt_mesh_pub_msh_vnd_range_client_pub_2_3 = NET_BUF_SIMPLE(3 + 4 + 4);

    // Configure buffers
    health_pub.msg = bt_mesh_pub_msg_health_pub;
    gen_onoff_server_pub.msg = bt_mesh_pub_msg_gen_onoff_server_pub;

    vnd_range_client_pub_1_1.msg = bt_mesh_pub_msh_vnd_range_client_pub_1_1;
    vnd_range_client_pub_1_2.msg = bt_mesh_pub_msh_vnd_range_client_pub_1_2;
    vnd_range_client_pub_1_3.msg = bt_mesh_pub_msh_vnd_range_client_pub_1_3;

    vnd_range_client_pub_2_1.msg = bt_mesh_pub_msh_vnd_range_client_pub_2_1;
    vnd_range_client_pub_2_2.msg = bt_mesh_pub_msh_vnd_range_client_pub_2_2;
    vnd_range_client_pub_2_3.msg = bt_mesh_pub_msh_vnd_range_client_pub_2_3;
}

uint8_t init_subscriber_sems(void)
{
    int ret;

    ret = os_sem_init(&receivedTagOne, 0);
    if (ret) {

        return ret;
    }

    ret = os_sem_init(&receivedTagTwo, 0);

    return ret;
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
 * @brief Vendor range info client status message handler
 * 
 * @param model Calling mesh model
 * @param ctx Message context
 * @param buf Message buffer
 */
static void vnd_range_status(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct os_mbuf *buf)
{

    int32_t r1 = (int32_t)net_buf_simple_pull_le32(buf);
    int32_t r2 = (int32_t)net_buf_simple_pull_le32(buf);

    switch (model->groups[0] & 0x000F) {
    case 0:
        testingRangeT1.r1 = r1;
        testingRangeT1.r2 = r2;
        break;
    case 1:
        testingRangeT1.r3 = r1;
        testingRangeT1.r4 = r2;
        break;
    case 2:
        testingRangeT1.r5 = r1;
        testingRangeT1.r6 = r2;

        os_sem_release(&receivedTagOne);
        break;
    case 3:
        testingRangeT2.r1 = r1;
        testingRangeT2.r2 = r2;
        break;
    case 4:
        testingRangeT2.r3 = r1;
        testingRangeT2.r4 = r2;
        break;
    case 5:
        testingRangeT2.r5 = r1;
        testingRangeT2.r6 = r2;

        os_sem_release(&receivedTagTwo);
        break;
    default:
        break;

    }
}

static const struct bt_mesh_model_op gen_onoff_server_opcodes[] = {
    {BT_MESH_MODEL_OP_2(0x82, 0x01), 0, gen_onoff_get},
    {BT_MESH_MODEL_OP_2(0x82, 0x02), 2, gen_onoff_set},
    {BT_MESH_MODEL_OP_2(0x82, 0x03), 2, gen_onoff_set_unack},
    BT_MESH_MODEL_OP_END
};

static const struct bt_mesh_model_op vnd_range_client_opcodes[] = {
    {VND_RANGE_MODEL_STATUS_OPCODE, 8, vnd_range_status},
    BT_MESH_MODEL_OP_END
};

// Setup of device root element models
struct bt_mesh_model root_models[] = {
    BT_MESH_MODEL_CFG_SRV(&config_server),
    BT_MESH_MODEL_HEALTH_SRV(&health_server, &health_pub),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_server_opcodes, &gen_onoff_server_pub, &onoff_server_led),
};

struct bt_mesh_model vnd_models_1[] = {
    BT_MESH_MODEL_VND(CID_RUNTIME, 0x4100, vnd_range_client_opcodes, &vnd_range_client_pub_1_1, NULL),
    BT_MESH_MODEL_VND(CID_RUNTIME, 0x4200, vnd_range_client_opcodes, &vnd_range_client_pub_1_2, NULL),
    BT_MESH_MODEL_VND(CID_RUNTIME, 0x4302, vnd_range_client_opcodes, &vnd_range_client_pub_1_3, NULL),
};

struct bt_mesh_model vnd_models_2[] = {
    BT_MESH_MODEL_VND(CID_RUNTIME, 0x4100, vnd_range_client_opcodes, &vnd_range_client_pub_2_1, NULL),
    BT_MESH_MODEL_VND(CID_RUNTIME, 0x4200, vnd_range_client_opcodes, &vnd_range_client_pub_2_2, NULL),
    BT_MESH_MODEL_VND(CID_RUNTIME, 0x4302, vnd_range_client_opcodes, &vnd_range_client_pub_2_3, NULL),
};


struct bt_mesh_model *led_onoff_server = &root_models[2];

// Setup of device elements
static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
    BT_MESH_ELEM(0, BT_MESH_MODEL_NONE, vnd_models_1),
    BT_MESH_ELEM(0, BT_MESH_MODEL_NONE, vnd_models_2),
};

// Device mesh composition
const struct bt_mesh_comp composition = {
    .cid = CID_RUNTIME,
    .elem = elements, 
    .elem_count = ARRAY_SIZE(elements)
};