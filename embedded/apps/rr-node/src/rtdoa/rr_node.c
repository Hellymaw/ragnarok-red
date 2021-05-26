
/**
 ************************************************************************
 * @file rr_node.c
 * @author Wilfred MK
 * @date 19.05.2021 (Last Updated)
 * @brief Application for setting up master and slave nodes for rtdoa
 *          ranging. 
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
#include <uwb/uwb_mac.h>
#include <uwb/uwb_ftypes.h>
#include "dw1000/dw1000_dev.h"
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
#if MYNEWT_VAL(NRNG_ENABLED)
#include <nrng/nrng.h>
#endif
#include <rtdoa/rtdoa.h>
#include <rtdoa_node/rtdoa_node.h>
#include <tofdb/tofdb.h>

//#define DIAGMSG(s,u) printf(s,u)
#ifndef DIAGMSG
#define DIAGMSG(s,u)
#endif


#if MYNEWT_VAL(UWB_PAN_ENABLED)
#include <uwb_pan/uwb_pan.h>

#include "panmaster/panmaster.h"

#endif
#include "rr_node.h"

// Has the UWB config been updated already
static bool uwb_config_updated = false;

/**
 * @brief Updates UWB Config
 * 
 * @return int returns 0
 */
int uwb_config_upd_cb()
{
    /* Workaround in case we're stuck waiting for ccp with the
     * wrong radio settings */
    struct uwb_dev * inst = uwb_dev_idx_lookup(0);
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

// TDMA slot event
static struct dpl_event slot_event;

/**
 * @brief TDMA slot complete callback
 * 
 * @param inst uwb dev instance
 * @param cbs uwb mac interface 
 * @return true success
 * @return false invalid range
 */
static bool complete_cb(struct uwb_dev * inst, struct uwb_mac_interface * cbs)
{

    // Check if distance is valid
    if(inst->fctrl != FCNTL_IEEE_RANGE_16){
    
        return false;
    }
    
    // TDMA slot completed so place slot event in the event queue
    dpl_eventq_put(dpl_eventq_dflt_get(), &slot_event);
    return true;
}

/**
 * @brief NRNG completed callback
 * 
 * DISCLAIMER: This is manufacturer code and does arcane magic
 * 
 * @param ev NRNG event
 */
static void nrng_complete_cb(struct dpl_event *ev)
{

    assert(ev != NULL);
    assert(dpl_event_get_arg(ev));

    struct nrng_instance *nrng = (struct nrng_instance *)dpl_event_get_arg(ev);
    nrng_frame_t *frame = nrng->frames[(nrng->idx) % nrng->nframes];

    for (int i = 0; i < nrng->nframes; i++) {
        
        frame = nrng->frames[(nrng->idx + i) % nrng->nframes];
        uint16_t dst_addr = frame->dst_address;

        if (frame->code != UWB_DATA_CODE_SS_TWR_NRNG_FINAL || 
                frame->seq_num != nrng->seq_num) {

            continue;
        }

        float tof = nrng_twr_to_tof_frames(nrng->dev_inst, frame, frame);
        tofdb_set_tof(dst_addr, tof);
    }
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
    tdma_slot_t *slot = (tdma_slot_t *)dpl_event_get_arg(ev);
    tdma_instance_t *tdma = slot->parent;
    struct uwb_ccp_instance *ccp = tdma->ccp;
    uint16_t idx = slot->idx;
    nmgr_uwb_instance_t *nmgruwb = (nmgr_uwb_instance_t *)slot->arg;
    assert(nmgruwb);

    /* Avoid colliding with the ccp */
    if (dpl_sem_get_count(&ccp->sem) == 0) {
        return;
    }

    uint16_t timeout = 3*ccp->period/tdma->nslots/4;
    if (uwb_nmgr_process_tx_queue(nmgruwb, tdma_tx_slot_start(tdma, idx)) == false) {
        nmgr_uwb_listen(nmgruwb, UWB_BLOCKING, tdma_rx_slot_start(tdma, idx), timeout);
    }
}

/**
 * @brief Node to Node ranging for tof timing compensation
 * 
 * DISCLAIMER: This is manufacturer code and does arcane magic
 *
 */
static void nrng_slot_timer_cb(struct dpl_event *ev)
{

    assert(ev);
    tdma_slot_t * slot = (tdma_slot_t *) dpl_event_get_arg(ev);;
    tdma_instance_t * tdma = slot->parent;
    struct uwb_dev * inst = tdma->dev_inst;
    struct uwb_ccp_instance * ccp = tdma->ccp;
    struct nrng_instance * nrng = (struct nrng_instance *) uwb_mac_find_cb_inst_ptr(inst, UWBEXT_NRNG);

    uint16_t idx = slot->idx;

    /* Avoid colliding with the ccp */
    if (dpl_sem_get_count(&ccp->sem) == 0 || idx == 0xffff) {
        return;
    }

    hal_gpio_write(LED_BLINK_PIN, 1);

    uint16_t anchor_rng_initiator = ccp->seq_num % 8;
    if (anchor_rng_initiator == inst->slot_id) {
        
        uint64_t dx_time = tdma_tx_slot_start(tdma, idx) & 0xFFFFFFFFFE00UL;
        uint32_t slot_mask = 0xFFFF;

        if(nrng_request_delay_start(nrng, UWB_BROADCAST_ADDRESS, dx_time, 
                UWB_DATA_CODE_SS_TWR_NRNG, slot_mask, 0).start_tx_error) {
            /* Do nothing */
        }
    } else {

        uwb_set_delay_start(inst, tdma_rx_slot_start(tdma, idx));
        uint16_t timeout = uwb_phy_frame_duration(inst, 
                sizeof(nrng_request_frame_t)) + nrng->config.rx_timeout_delay;

        uwb_set_rx_timeout(inst, timeout + 0x100);
        nrng_listen(nrng, UWB_BLOCKING);
    }

    hal_gpio_write(LED_BLINK_PIN, 0);
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
    uint16_t idx = slot->idx;
    tdma_instance_t * tdma = slot->parent;
    assert(tdma);
    struct uwb_ccp_instance * ccp = tdma->ccp;
    assert(ccp);
    struct uwb_dev * inst = tdma->dev_inst;
    assert(inst);
    struct rtdoa_instance* rtdoa = (struct rtdoa_instance*)slot->arg;
    assert(rtdoa);

    /* Avoid colliding with the ccp */
    if (dpl_sem_get_count(&ccp->sem) == 0) {

        return;
    }

    if (uwb_config_updated) {

        uwb_mac_config(inst, NULL);
        uwb_txrf_config(inst, &inst->config.txrf);
        uwb_config_updated = false;
        return;
    }

    /* See if there's anything to send, if so finish early */
    nmgr_uwb_instance_t *nmgruwb = (nmgr_uwb_instance_t*)uwb_mac_find_cb_inst_ptr(tdma->dev_inst, UWBEXT_NMGR_UWB);
    assert(nmgruwb);
    if (uwb_nmgr_process_tx_queue(nmgruwb, tdma_tx_slot_start(tdma, idx)) == 
            true) {
        
        return;
    }

    if (inst->role & UWB_ROLE_CCP_MASTER) {

        uint64_t dx_time = tdma_tx_slot_start(tdma, idx) & 0xFFFFFFFFFE00UL;

        if (rtdoa_request(rtdoa, dx_time).start_tx_error) {

            /* Do nothing */
            printf("rtdoa_start_err\n");
        }
    } else {

        uint64_t dx_time = tdma_rx_slot_start(tdma, idx);
        if (rtdoa_listen(rtdoa, UWB_BLOCKING, dx_time, 3*ccp->period/tdma->nslots/4).start_rx_error) {
            printf("#rse\n");
        }
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
    struct uwb_dev * inst = tdma->dev_inst;

    /* Pan is slot 1 */
    struct uwb_pan_instance *pan = (struct uwb_pan_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_PAN);
    assert(pan);
    tdma_assign_slot(tdma, uwb_pan_slot_timer_cb, 1, (void*)pan);

    // anchor-to-anchor range slot is 31
    struct nrng_instance * nrng = (struct nrng_instance *)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_NRNG);
    assert(nrng);
    tdma_assign_slot(tdma, nrng_slot_timer_cb, 31, (void*)nrng);

    nmgr_uwb_instance_t *nmgruwb = (nmgr_uwb_instance_t*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_NMGR_UWB);
    assert(nmgruwb);
    struct rtdoa_instance* rtdoa = (struct rtdoa_instance*)uwb_mac_find_cb_inst_ptr(inst, UWBEXT_RTDOA);
    assert(rtdoa);

    for (i = 2; i < MYNEWT_VAL(TDMA_NSLOTS); i++) {
        
        if (i == 31) {
        
            continue;
        }
        if (i % 12 == 0) {
            
            tdma_assign_slot(tdma, nmgr_slot_timer_cb, i, (void*)nmgruwb);
        } else {
            
            tdma_assign_slot(tdma, rtdoa_slot_timer_cb, i, (void*)rtdoa);
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
void rtdoa_node_task(void *arg) 
{

    // Initialise LED that indicates UWB activity/sync
    hal_gpio_init_out(LED_BLINK_PIN, 1);

    // Register a configuration callback and load configuration
    uwbcfg_register(&uwb_cb);
    conf_load();

    // Setup UWB MAC interface
    struct uwb_mac_interface cbs = (struct uwb_mac_interface){
        .id =  UWBEXT_APP0,
        .complete_cb = complete_cb
    };

    // Get UWB device pointer
    struct uwb_dev *udev = uwb_dev_idx_lookup(0);
    
    // Append UWB device pointer to MAC interface
    uwb_mac_append_interface(udev, &cbs);

    // Get NRNG instance
    struct nrng_instance *nrng = (struct nrng_instance *)uwb_mac_find_cb_inst_ptr(udev, UWBEXT_NRNG);
    assert(nrng);

    // Initialise the NRNG completed event
    dpl_event_init(&slot_event, nrng_complete_cb, nrng);

    // Setup UWB device configuration
    udev->config.rxauto_enable = 0;
    udev->config.trxoff_enable = 1;
    udev->config.rxdiag_enable = 1;
    udev->config.sleep_enable = 1;
    udev->config.dblbuffon_enabled = 0;
    uwb_set_dblrxbuff(udev, false);
    udev->slot_id = 0xffff;

    // Get CCP instance
    struct uwb_ccp_instance *ccp = (struct uwb_ccp_instance *) uwb_mac_find_cb_inst_ptr(udev, UWBEXT_CCP);
    assert(ccp);

    // Get PAN instance
    struct uwb_pan_instance *pan = (struct uwb_pan_instance *) uwb_mac_find_cb_inst_ptr(udev, UWBEXT_PAN);
    assert(pan);

    // If node is a master, configure to be a master
    if (udev->role & UWB_ROLE_CCP_MASTER) {

        printf("{\"role\":\"ccp_master\"}\n");

        // Start sending CCP
        uwb_ccp_start(ccp, CCP_ROLE_MASTER);

        // Seems that this sets up for over the air upgrade
        struct image_version fw_ver;
        struct panmaster_node *node;
        panmaster_idx_find_node(udev->euid, NETWORK_ROLE_ANCHOR, &node);
        assert(node);
        imgr_my_version(&fw_ver);
        node->fw_ver.iv_major = fw_ver.iv_major;
        node->fw_ver.iv_minor = fw_ver.iv_minor;
        node->fw_ver.iv_revision = fw_ver.iv_revision;
        node->fw_ver.iv_build_num = fw_ver.iv_build_num;
        udev->my_short_address = node->addr;
        udev->slot_id = node->slot_id;
        panmaster_postprocess();

        // Start panning
        uwb_pan_start(pan, UWB_PAN_ROLE_MASTER, NETWORK_ROLE_ANCHOR);
    } else {

        // If slave, start CCP and PAN as slave
        uwb_ccp_start(ccp, CCP_ROLE_RELAY);
        uwb_pan_start(pan, UWB_PAN_ROLE_RELAY, NETWORK_ROLE_ANCHOR);
    }

    // Print some useful UWB device info
    printf("{\"device_id\":\"%lX\"",udev->device_id);
    printf(",\"panid\":\"%X\"",udev->pan_id);
    printf(",\"addr\":\"%X\"",udev->uid);
    printf(",\"part_id\":\"%lX\"",(uint32_t)(udev->euid&0xffffffff));
    printf(",\"lot_id\":\"%lX\"}\n",(uint32_t)(udev->euid>>32));

    // Get next TDMA instance
    tdma_instance_t * tdma = (tdma_instance_t*)uwb_mac_find_cb_inst_ptr(udev, UWBEXT_TDMA);
    assert(tdma);

    // Allocate TDMA slots
    tdma_allocate_slots(tdma);

#if MYNEWT_VAL(NCBWIFI_ESP_PASSTHROUGH)
    hal_bsp_esp_bypass(true);
#endif

    while(1) {

        os_time_delay(OS_TICKS_PER_SEC);
    }
    
    return;
}
