#ifndef NEW_DEV_COMP_H
#define NEW_DEV_COMP_H

#include "bsp/bsp.h"
#include <os/os.h>
#include "mesh/mesh.h"

#define CID_RUNTIME 0x05C3 // TODO: change company ID

#define VND_RANGE_MODEL_GET_OPCODE      BT_MESH_MODEL_OP_3(0x01, CID_RUNTIME)
#define VND_RANGE_MODEL_STATUS_OPCODE   BT_MESH_MODEL_OP_3(0x02, CID_RUNTIME)

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

struct uwbData {
    uint8_t tagNum;
    int32_t r1;
    int32_t r2;
    int32_t r3;
    int32_t r4;
    int32_t r5;
    int32_t r6;
};

extern struct os_sem receivedTagOne;

extern struct os_sem receivedTagTwo;

extern struct uwbData testingRangeT1;

extern struct uwbData testingRangeT2;

extern struct bt_mesh_model *led_onoff_server;

extern struct bt_mesh_model root_models[];

extern struct bt_mesh_model vnd_models_1[];

extern struct bt_mesh_model vnd_models_2[];

extern const struct bt_mesh_comp composition;

void init_pub(void);

uint8_t init_subscriber_sems(void);

#endif