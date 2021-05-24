#ifndef NEW_DEV_COMP_H
#define NEW_DEV_COMP_H

#include "bsp/bsp.h"
#include <os/os.h>
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

// Stores ranging data for UWB
struct uwbData {
    uint8_t tagNum;
    int32_t r1;
    int32_t r2;
    int32_t r3;
    int32_t r4;
    int32_t r5;
    int32_t r6;
};

// Semaphore to synchronise on receiving data from tag 1
extern struct os_sem receivedTagOne;

// Semaphore to synchronise on receiving data from tag 1
extern struct os_sem receivedTagTwo;

// Stores tag 1 ranging data
extern struct uwbData testingRangeT1;

// Stores tag 2 ranging data
extern struct uwbData testingRangeT2;

// LED OnOff server
extern struct bt_mesh_model *led_onoff_server;

// Root element models
extern struct bt_mesh_model root_models[];

// Vendor models for element 2 (tag 1)
extern struct bt_mesh_model vnd_models_1[];

// Vendor models for element 3 (tag 2)
extern struct bt_mesh_model vnd_models_2[];

// Device BLE mesh composition
extern const struct bt_mesh_comp composition;

// Initialise publisher buffers
void init_pub(void);

// Initialises semaphores used to synchronise output of ranging data
uint8_t init_subscriber_sems(void);

#endif