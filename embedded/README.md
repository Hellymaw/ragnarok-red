<!--
#
# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
#  KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.
#
-->

# Ragnarok Red

## Overview

This is the mynewt, nimble, decawave-uwb based embedded code for the Ragnarok Red project

## Building

After cloning the repositry, call
```
    $ newt upgrade
```
### Applying Patches

```
    $ repos/decawave-uwb-core/setup.sh
    $ newt upgrade
    
    cd repos/apache-mynewt-core/
    git apply ../decawave-uwb-core/patches/apache-mynewt-core/mynewt_1_7_0_*
    cd -
```
To retrieve all project repository dependencies. Then:


1. Build bootloader application for the DWM1001-dev target.
(executed from the embedded directory)

Erase board flash: 
This required NRF CLI tools
```
    $ nrfjprog -f NRF52 -e
```
alternatively do

```
$ JLinkExe -device nRF52 -speed 4000 -if SWD
J-Link>erase
J-Link>exit
```

```
    newt target create dwm1001_boot
    newt target set dwm1001_boot app=@mcuboot/boot/mynewt
    newt target set dwm1001_boot bsp=@decawave-uwb-core/hw/bsp/dwm1001
    newt target set dwm1001_boot build_profile=optimized
    newt build dwm1001_boot
    newt load dwm1001_boot
```

2. Setup target for rr-node

```
    newt target create rr_node
    newt target set rr_node app=apps/rr-node
    newt target set rr_node bsp=@decawave-uwb-core/hw/bsp/dwm1001
    newt target set rr_node build_profile=optimized
    newt run rr_node 0
    
```

3. Setup target for rr-tag

```
    newt target create rr_tag
    newt target set rr_tag app=apps/rr-node
    newt target set rr_tag bsp=@decawave-uwb-core/hw/bsp/dwm1001
    newt target set rr_tag build_profile=optimized
```

4. Build and run on relevent target

```
    newt run rr_node 0
```

or:

```
    newt run rr_tag 0
```

## Handy BLE mesh things

1. To help setup `meshctl` correcly:
    https://www.bluetooth.com/wp-content/uploads/2020/04/Developer-Study-Guide-How-to-Deploy-BlueZ-on-a-Raspberry-Pi-Board-as-a-Bluetooth-Mesh-Provisioner.pdf

2. To help provision devices using `meshctl`:
    https://3pl46c46ctx02p7rzdsvsg21-wpengine.netdna-ssl.com/wp-content/uploads/2019/03/Tutorial-How-to-set-up-BlueZ_Part2-3.pdf
