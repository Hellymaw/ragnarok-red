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
    if (dpl_sem_get_count(&ccp->sem) == 0 || !ccp->status.valid) {
        uwb_mac_config(inst, NULL);
        uwb_txrf_config(inst, &inst->config.txrf);
        if (dpl_sem_get_count(&ccp->sem) == 0) {
            uwb_start_rx(inst);
        }
        return 0;
    }
    uwb_config_updated = true;
    return 0;
}

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
void rr_tag_packet(struct rtdoabh_tag_results_pkg *p) {
 
    // printf("--------\n");
    for (int i=0;i<p->num_ranges;i++) {
        struct rtdoabh_range_data *r = &p->ranges[i];
        //int sign = (r->diff_dist_mm > 0);
        float ddist_m  = r->diff_dist_mm/1000;
        int ddist_mm = abs(r->diff_dist_mm - ddist_m*1000);

        //TODO Dont print masters range
        if (r->anchor_addr != MASTER_NODE_ID) {
           //printf("Node [%x] -> %s%d.%03d\n", r->anchor_addr,(sign)?"":"-", abs(ddist_m), ddist_mm);
            //printf("Node [%x] : %d.%03d\n", r->anchor_addr,abs(ddist_m), ddist_mm);
            switch (get_node_index(r->anchor_addr)) {
            case 1:
                ranges1.r1 = ddist_mm;
                break;
            case 2:
                ranges1.r2 = ddist_mm;
                break;
            case 3:
                ranges2.r1 = ddist_mm;
                break;
            case 4:
                ranges2.r2 = ddist_mm;
                break;
            case 5:
                ranges3.r1 = ddist_mm;
                break;
            case 6:
                ranges3.r2 = ddist_mm;
                break;
            default:
                break;
            }
        }
    }
    vnd_range_publish(&vnd_models[0]);
    vnd_range_publish(&vnd_models[1]);
    vnd_range_publish(&vnd_models[2]);
    // printf("--------\n");
}


/**
 * @fn rtdoa_slot_timer_cb(struct dpl_event * ev)
 *
 * @brief RTDoA Subscription slot
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
    //printf("idx%d\n", idx);

    /* Avoid colliding with the ccp */
    if (dpl_sem_get_count(&ccp->sem) == 0) {
        return;
    }
    hal_gpio_write(LED_BLINK_PIN,1);
    uint64_t dx_time = tdma_rx_slot_start(tdma, idx);
    if(rtdoa_listen(rtdoa, UWB_BLOCKING, dx_time, 3*ccp->period/tdma->nslots/4).start_rx_error) {
        printf("#rse\n");
    }
    hal_gpio_write(LED_BLINK_PIN,0);

    if (dpl_sem_get_count(&ccp->sem) == 0) {
        return;
    }
    uint64_t measurement_ts = uwb_wcs_local_to_master64(ccp->wcs, dx_time);
    rtdoa_backhaul_set_ts(measurement_ts>>16);
    //This Causes a callback through the backend to rr_tag_packet
    rtdoa_backhaul_send(inst, rtdoa, 0); //tdma_tx_slot_start(inst, idx+2)
    //printf("idx%de\n", idx);
}

/**
 * @brief Slot timer callback
 * 
 * @param ev  callback event
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
    // printf("idx %02d nmgr\n", idx);

    if (uwb_config_updated) {
        uwb_mac_config(inst, NULL);
        uwb_txrf_config(inst, &inst->config.txrf);
        uwb_config_updated = false;
    }

    /* Avoid colliding with the ccp */
    if (dpl_sem_get_count(&ccp->sem) == 0) {
        return;
    }

    if (uwb_nmgr_process_tx_queue(nmgruwb, tdma_tx_slot_start(tdma, idx)) == false) {
        nmgr_uwb_listen(nmgruwb, UWB_BLOCKING, tdma_rx_slot_start(tdma, idx),
             3*ccp->period/tdma->nslots/4);
    }
}

/**
 * @brief Allocates TDMA Slots to device
 * 
 * @param tdma 
 */
static void tdma_allocate_slots(tdma_instance_t * tdma)
{
    uint16_t i;
    /* Pan for anchors is in slot 1 */
    struct uwb_dev * inst = tdma->dev_inst;
    nmgr_uwb_instance_t *nmgruwb = (nmgr_uwb_instance_t*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_NMGR_UWB);
    assert(nmgruwb);
    struct rtdoa_instance * rtdoa = (struct rtdoa_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_RTDOA);
    assert(rtdoa);

    /* anchor-to-anchor range slot is 31 */
    for (i=2;i < MYNEWT_VAL(TDMA_NSLOTS);i++) {
        if (i==31) {
            continue;
        }
        if (i%12==0) {
            tdma_assign_slot(tdma, nmgr_slot_timer_cb, i, nmgruwb);
        } else {
            tdma_assign_slot(tdma, rtdoa_slot_timer_cb, i, rtdoa);
        }
    }
}


/**
 * @brief Primary thread that will carry out RTDOA functionality.
 * 
 * @param arg 
 */
void rtdoa_tag_task(void *arg) {
    //Setup UWB
    hal_gpio_init_out(LED_BLINK_PIN, 1);
    uwbcfg_register(&uwb_cb);
    conf_load();

    struct uwb_dev *udev = uwb_dev_idx_lookup(0);

    udev->config.rxauto_enable = 1;
    udev->config.trxoff_enable = 1;
    udev->config.rxdiag_enable = 1;
    udev->config.sleep_enable = 1;
    udev->config.dblbuffon_enabled = 0;
    uwb_set_dblrxbuff(udev, false);

    udev->slot_id = 0;

#if MYNEWT_VAL(BLEPRPH_ENABLED)
    printf("bleprph is enabled\n");
#endif

    // ble_init(udev->my_long_address);

    printf("{\"device_id\"=\"%lX\"",udev->device_id);
    printf(",\"panid=\"%X\"",udev->pan_id);
    printf(",\"addr\"=\"%X\"",udev->uid);
    printf(",\"part_id\"=\"%lX\"",(uint32_t)(udev->euid&0xffffffff));
    printf(",\"lot_id\"=\"%lX\"}\n",(uint32_t)(udev->euid>>32));

    struct uwb_ccp_instance * ccp = (struct uwb_ccp_instance*)uwb_mac_find_cb_inst_ptr(udev, UWBEXT_CCP);
    assert(ccp);
    tdma_instance_t * tdma = (tdma_instance_t*)uwb_mac_find_cb_inst_ptr(udev, UWBEXT_TDMA);
    assert(tdma);

    tdma_allocate_slots(tdma);
    uwb_ccp_start(ccp, CCP_ROLE_SLAVE);
    rtdoa_backhaul_set_role(udev, RTDOABH_ROLE_BRIDGE);


    while(1) {
        // dpl_eventq_run(dpl_eventq_dflt_get());
        os_time_delay(OS_TICKS_PER_SEC);
    }

    assert(0);
    return;
}


