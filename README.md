# Required Changes
Note that this will cause image upgrades to fail.

Changes to `embedded/repos/decawave-uwb-core/hw/bsp/dwm1001/bsp.yml`:
```
FLASH_AREA_IMAGE_0:
    device: 0
    offset: 0x00008000
    size: 432kB
FLASH_AREA_IMAGE_1:
    device: 0
    offset: 0x00074000
    size: 32kB
```

Changes to `embedded/repos/decawave-uwb-core/hw/bsp/dwm1001/nrf52xxaa.ld`:
```
MEMORY
{
  FLASH (rx) : ORIGIN = 0x00008000, LENGTH = 0x6c000
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x10000
}
```
