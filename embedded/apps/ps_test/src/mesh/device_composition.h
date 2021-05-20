#ifndef NEW_DEV_COMP_H
#define NEW_DEV_COMP_H

#include "bsp/bsp.h"
#include "mesh/mesh.h"

#define CID_RUNTIME 0x05C3 // TODO: change company ID

#define VND_RANGE_MODEL_GET_OPCODE BT_MESH_MODEL_OP_3(0x01, CID_RUNTIME)
#define VND_RANGE_MODEL_STATUS_OPCODE BT_MESH_MODEL_OP_3(0x02, CID_RUNTIME)

struct onoff_state {
    uint8_t current;
    uint8_t previous;
    uint8_t led_gpio_pin;
};

// TODO: add extra ranges
struct tag_ranges {
    int32_t r1;
    int32_t r2;
};

extern struct bt_mesh_model *led_onoff_server;

extern struct tag_ranges ranges;

extern struct bt_mesh_model root_models[];

extern struct bt_mesh_model vnd_models[];

extern const struct bt_mesh_comp composition;

void init_pub(void);

void randomise_publishers_TID(void);

void gen_onoff_set_unack(struct bt_mesh_model *, struct bt_mesh_msg_ctx *, struct os_mbuf *);

#endif