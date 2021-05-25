/**
 ************************************************************************
 * @file rr_tag.c
 * @author Wilfred MK
 * @date 19.05.2021 (Last Updated)
 * @brief Initialises the device as tag and carries out RTDOA funcitonality
 *          within the tag. 
 **********************************************************************
 **/
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "sysinit/sysinit.h"
#include "os/os.h"
#include "bsp/bsp.h"
#include "imgmgr/imgmgr.h"
#include "hal/hal_gpio.h"
#include "hal/hal_bsp.h"
#include <hal/hal_system.h>
#ifdef ARCH_sim
#include "mcu/mcu_sim.h"
#endif

#if MYNEWT_VAL(BLEPRPH_ENABLED)
#include "bleprph/bleprph.h"
#endif

#include <uwb/uwb.h>
#include "uwb/uwb_ftypes.h"
#include "dw1000/dw1000_hal.h"
#include "uwbcfg/uwbcfg.h"
#include <config/config.h>

#if MYNEWT_VAL(NMGR_UWB_ENABLED)
#include <nmgr_uwb/nmgr_uwb.h>
#endif

#include <tdma/tdma.h>
#include <uwb_ccp/uwb_ccp.h>
#include <uwb_wcs/uwb_wcs.h>
#include <timescale/timescale.h>
#if MYNEWT_VAL(UWB_RNG_ENABLED)
#include <uwb_rng/uwb_rng.h>
#endif
#include <rtdoa/rtdoa.h>

#include "mesh/mesh.h"
#include "mesh/ble_mesh.h"
#include <rtdoa_backhaul/rtdoa_backhaul.h>

#include "rr_tag.h"

#include "mesh/device_composition.h"

// Has the UWB config been updated already
static bool uwb_config_updated = false;

/**
 * @brief Updates the UWB Config 
 * 
 * @return int 0 on success.
 */
static int uwb_config_upd_cb()
{
    /* Workaround in case we're stuck waiting for ccp with the
     * wrong radio settings */
    struct uwb_dev *inst = uwb_dev_idx_lookup(0);
    struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_CCP);

    // If no released semaphores and status is not valid, reconfigure
    if (dpl_sem_get_count(&ccp->sem) == 0 || !ccp->status.valid) {
    
        uwb_mac_config(inst, NULL);
        uwb_txrf_config(inst, &inst->config.txrf);
    
        // Start receiving with UWB if no semaphores
        if (dpl_sem_get_count(&ccp->sem) == 0) {
    
            uwb_start_rx(inst);
        }
        return 0;
    }

    // Config was updated
    uwb_config_updated = true;
    return 0;
}

// Set UWB config update callback
struct uwbcfg_cbs uwb_cb = {
    .uc_update = uwb_config_upd_cb
};

/**
 * @brief Return a node LUT index from a given slave node address.
 *        This address is based on MAC and will not change (unique per slave)
 * 
 * @param nodeAddr slave nodes unique address
 * @return uint8_t LUT index val
 */
uint8_t get_node_index(int nodeAddr) {

    switch(nodeAddr) {
        case SLAVE_NODE1_ID:
            return 1;
        case SLAVE_NODE2_ID:
            return 2;
        case SLAVE_NODE3_ID:
            return 3;
        case SLAVE_NODE4_ID:
            return 4;
        case SLAVE_NODE5_ID:
            return 5;
        case SLAVE_NODE6_ID:
            return 6;
        default:
            //Should not be here
            return -1;
    }
}

/**
 * @brief Function to handle a tag packet, this is called in rtdoa_backhaul.c
 *          to get backend data over to this file.
 *        See, readme.md for configuring respective lib file.
 * 
 * @param p 
 */
void rr_tag_packet(struct rtdoabh_tag_results_pkg *p) 
{
 
    // Get ranges for all connected slaves
    for (int i = 0; i < p->num_ranges; i++) {

        struct rtdoabh_range_data *r = &p->ranges[i];
        
        // Don't save range if master
        if (r->anchor_addr != MASTER_NODE_ID) {
           
            // Save range to correct Vendor Range Model
            switch (get_node_index(r->anchor_addr)) {
            case 1:
                ranges1.r1 = r->diff_dist_mm;
                break;
            case 2:
                ranges1.r2 = r->diff_dist_mm;
                break;
            case 3:
                ranges2.r1 = r->diff_dist_mm;
                break;
            case 4:
                ranges2.r2 = r->diff_dist_mm;
                break;
            case 5:
                ranges3.r1 = r->diff_dist_mm;
                break;
            case 6:
                ranges3.r2 = r->diff_dist_mm;
                break;
            default:
                break;
            }
        }
    }
}


/**
 * @brief RTDoA Emission slot
 *
 * DISCLAIMER: This is manufacturer code and does arcane magic
 * 
 */
static void rtdoa_slot_timer_cb(struct dpl_event *ev)
{

    assert(ev);
    tdma_slot_t * slot = (tdma_slot_t *) dpl_event_get_arg(ev);
    tdma_instance_t * tdma = slot->parent;
    struct uwb_ccp_instance * ccp = tdma->ccp;
    struct uwb_dev * inst = tdma->dev_inst;
    uint16_t idx = slot->idx;
    struct rtdoa_instance * rtdoa = (struct rtdoa_instance*)slot->arg;
    
    /* Avoid colliding with the ccp */
    if (dpl_sem_get_count(&ccp->sem) == 0) {
    
        return;
    }
    
    hal_gpio_write(LED_BLINK_PIN,1);
    
    uint64_t dx_time = tdma_rx_slot_start(tdma, idx);
    
    // Try get a packet from a slave
    if(rtdoa_listen(rtdoa, UWB_BLOCKING, dx_time, 
            3*ccp->period/tdma->nslots/4).start_rx_error) {
        
        printf("#rse\n");
    }
    
    hal_gpio_write(LED_BLINK_PIN,0);

    if (dpl_sem_get_count(&ccp->sem) == 0) {
    
        return;
    }
    
    uint64_t measurement_ts = uwb_wcs_local_to_master64(ccp->wcs, dx_time);
    rtdoa_backhaul_set_ts(measurement_ts>>16);
    
    //This Causes a callback through the backend to rr_tag_packet
    rtdoa_backhaul_send(inst, rtdoa, 0);
}

/**
 * @brief NewtMGR slot timer callback
 *        I think this is used for over the air upgrade but I'm unsure
 * 
 * DISCLAIMER: This is manufacturer code and does arcane magic
 * 
 * @param ev NewtMGR event
 */
static void nmgr_slot_timer_cb(struct dpl_event * ev)
{

    assert(ev);
    tdma_slot_t * slot = (tdma_slot_t *) dpl_event_get_arg(ev);
    tdma_instance_t * tdma = slot->parent;
    struct uwb_ccp_instance *ccp = tdma->ccp;
    struct uwb_dev * inst = tdma->dev_inst;
    uint16_t idx = slot->idx;
    nmgr_uwb_instance_t * nmgruwb = (nmgr_uwb_instance_t *)slot->arg;
    assert(nmgruwb);
    
    // If UWB config was updated, setup configs
    if (uwb_config_updated) {

        uwb_mac_config(inst, NULL);
        uwb_txrf_config(inst, &inst->config.txrf);
        uwb_config_updated = false;
    }

    // Avoid colliding with the ccp
    if (dpl_sem_get_count(&ccp->sem) == 0) {
        
        return;
    }

    if (uwb_nmgr_process_tx_queue(nmgruwb, tdma_tx_slot_start(tdma, idx)) == 
            false) {
        
        nmgr_uwb_listen(nmgruwb, UWB_BLOCKING, tdma_rx_slot_start(tdma, idx), 
                 3*ccp->period/tdma->nslots/4);
    }
}

/**
 * @brief Allocate TDMA Slots
 * 
 * DISCLAIMER: This is manufacturer code and does arcane magic
 * 
 * @param TDMA instance
 */
static void tdma_allocate_slots(tdma_instance_t * tdma)
{

    uint16_t i;
    
    // Pan for anchors is in slot 1 
    struct uwb_dev * inst = tdma->dev_inst;
    nmgr_uwb_instance_t *nmgruwb = (nmgr_uwb_instance_t*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_NMGR_UWB);
    assert(nmgruwb);
    struct rtdoa_instance * rtdoa = (struct rtdoa_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_RTDOA);
    assert(rtdoa);

    // anchor-to-anchor range slot is 31
    for (i = 2; i < MYNEWT_VAL(TDMA_NSLOTS); i++) {
        
        if (i == 31) {
        
            continue;
        }
        
        if (i % 12 == 0) {
        
            tdma_assign_slot(tdma, nmgr_slot_timer_cb, i, nmgruwb);
        } else {
        
            tdma_assign_slot(tdma, rtdoa_slot_timer_cb, i, rtdoa);
        }
    }
}


/**
 * @brief Primary thread that will carry out RTDOA node functionality,
 *        based on build command, this will build for either a slave
 *        or a master node. 
 * 
 * @param arg N/A
 */
void rtdoa_tag_task(void *arg) 
{
    
    hal_gpio_init_out(LED_BLINK_PIN, 1);
    
    // Register a configuration callback and load configuration
    uwbcfg_register(&uwb_cb);
    conf_load();

    // Get UWB device pointer
    struct uwb_dev *udev = uwb_dev_idx_lookup(0);

    // Setup UWB device configuration
    udev->config.rxauto_enable = 1;
    udev->config.trxoff_enable = 1;
    udev->config.rxdiag_enable = 1;
    udev->config.sleep_enable = 1;
    udev->config.dblbuffon_enabled = 0;
    uwb_set_dblrxbuff(udev, false);
    udev->slot_id = 0;

    // Print some useful UWB device info
    printf("{\"device_id\"=\"%lX\"",udev->device_id);
    printf(",\"panid=\"%X\"",udev->pan_id);
    printf(",\"addr\"=\"%X\"",udev->uid);
    printf(",\"part_id\"=\"%lX\"",(uint32_t)(udev->euid&0xffffffff));
    printf(",\"lot_id\"=\"%lX\"}\n",(uint32_t)(udev->euid>>32));

    // Get CCP instance
    struct uwb_ccp_instance * ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(udev, UWBEXT_CCP);
    assert(ccp);

    // Get next TDMA instance
    tdma_instance_t * tdma = (tdma_instance_t*)uwb_mac_find_cb_inst_ptr(udev, UWBEXT_TDMA);
    assert(tdma);

    // Allocate TDMA slots
    tdma_allocate_slots(tdma);

    // Start looking for CCP 
    uwb_ccp_start(ccp, CCP_ROLE_SLAVE);

    // Set role in the RTDoA network
    rtdoa_backhaul_set_role(udev, RTDOABH_ROLE_BRIDGE);

    while(1) {
        
        os_time_delay(OS_TICKS_PER_SEC);
    }

    return;
}


