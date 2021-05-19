# Modification Required For Building

## rtdoa_backhaul.c
This is lib file located in the repos folder
>repos/decawave-uwb-apps/lib/rtdoa_backhaul/src/rtdoa_backhaul.c
### Changes needed:
Include the following header file at the top
> #include "rr_tag.h"

In function static void process_rx_data_queue(struct os_event *ev)
>line 362: comment out:  //rtdoa_backhaul_print(&pkg, true);

>line 363: add: rr_tag_packet(&pkg);


## Notes
This runs without the additional config options and pckg includes that are present in the primary repo, there was an issue with unhandled interrupts when those were enabled. Thus, this builds with settings only required for the RTDOA sample.