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

In function rtdoa_backhaul_send(struct uwb_dev * inst, struct rtdoa_instance *rtdoa,
                    uint64_t dx_time)

>line 642: Uncomment entire block commented out by default.

>        if (g_result_pkg.ref_anchor_addr == rtdoa->frames[i]->src_address) {
>           continue;
>        }


This skips frame difference calculation for the master/ref anchors.


## Notes
This runs without the additional config options and pckg includes that are present in the primary repo, there was an issue with unhandled interrupts when those were enabled. Thus, this builds with settings only required for the RTDOA sample.

## Nodes Status (Deprecicated)
C1: Master - C439 

D3: Slave - 19AB LEFT

D1: Slave - 8817 RIGHT

C3: Slave - 5132 TOP


## References 
RTDOA References: 
https://arxiv.org/pdf/2008.04248.pdf
