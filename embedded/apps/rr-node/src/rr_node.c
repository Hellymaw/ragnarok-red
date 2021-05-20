
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



/**
 * @brief Primary thread that will carry out RTDOA node functionality,
 *          based on build command, this will build for either a slave
 *          or a master node. 
 * 
 * @param arg 
 */
void rtdoa_node_task(void *arg) {

    while(1) {
        os_time_delay(OS_TICKS_PER_SEC/2);
        printf("Node\n");
    }

}

/**
 * @brief Task that will blink the board LED (Used for debug purposes)
 * 
 * @param arg 
 */
void blink_led_task(void *arg) {

    int led_pin = LED_1;
    hal_gpio_init_out(led_pin, 1);

    while(1) {
        os_time_delay(OS_TICKS_PER_SEC/2);
        /* Toggle the LED */
        hal_gpio_toggle(led_pin);
    }
}