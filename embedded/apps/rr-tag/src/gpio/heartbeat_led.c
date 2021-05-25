/**
 ************************************************************************
 * @file heartbeat_led.c
 * @author Aaron Helmore
 * @date 24.05.2021 (Last Updated)
 * @brief Contains a task that will blink the heartbeat LED.
 **********************************************************************
 **/

#include "heartbeat_led.h"

#include "os/os.h"
#include "bsp/bsp.h"
#include "hal/hal_gpio.h"
#include "hal/hal_bsp.h"


/**
 * @brief Task that will blink the board LED (Used for debug purposes)
 * 
 * @param arg 
 */
void heartbeat_led_task(void *arg) 
{

    // Initialise heartbeat LED
    hal_gpio_init_out(LED_4, 1);

    while(1) {

        // 2 Hz toggle
        os_time_delay(OS_TICKS_PER_SEC/2);
        hal_gpio_toggle(LED_4);
    }
}