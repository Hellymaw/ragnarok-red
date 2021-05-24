#ifndef NEW_DEV_COMP_H
#define NEW_DEV_COMP_H

#include "bsp/bsp.h"
#include "mesh/mesh.h"

// Company ID
#define CID_RUNTIME 0x05C3

// Vendor Model Get Opcode
#define VND_RANGE_MODEL_GET_OPCODE      BT_MESH_MODEL_OP_3(0x01, CID_RUNTIME)

// Vendor Model Status Opcode
#define VND_RANGE_MODEL_STATUS_OPCODE   BT_MESH_MODEL_OP_3(0x02, CID_RUNTIME)

// Stores state of Generic OnOff model
struct onoff_state {
    uint8_t current;
    uint8_t previous;
    uint8_t led_gpio_pin;
};

// Stores state of Vendor Range model
struct tag_ranges {
    int32_t r1;
    int32_t r2;
};

// LED OnOff server
extern struct bt_mesh_model *led_onoff_server;

// Slave node 1 & 2 ranges
extern struct tag_ranges ranges1;

// Slave node 3 & 4 ranges
extern struct tag_ranges ranges2;

// Slave node 5 & 6 ranges
extern struct tag_ranges ranges3;

// Root element models
extern struct bt_mesh_model root_models[];

// Vendor models for element 2
extern struct bt_mesh_model vnd_models[];

// Device BLE mesh composition
extern const struct bt_mesh_comp composition;

// Initialise publisher buffers
void init_pub(void);

// Randomise the TID for messages
void randomise_publishers_TID(void);

// Publish Vendor Range model
void vnd_range_publish(struct bt_mesh_model *);

#endif