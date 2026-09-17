# Hardware

## Boards

| | |
|---|---|
| Compute | NVIDIA Jetson Orin Nano Developer Kit |
| Panda | FK407M1 core board, **STM32F407VET6** @ 168 MHz, 512K flash, 128K contiguous SRAM |
| Camera | IMX477 on CSI **CAM1** |
| CAN | 2 × transceiver, one per bus |
| GPS | serial receiver on `ttyTHS1` |
| IMU | BNO085 |

The panda began as a Nucleo-F446RE talking over the ST-Link virtual COM port. It is now an
FK407M1 with native USB, which is why there is no serial device at all and the host
transport was rewritten. Both board targets still build.

## Settings that differ from the default

Every one of these was wrong at its default and cost time to find.

| setting | value | note |
|---|---|---|
| `OP_SENSOR_ID` | `0` | Argus enumeration; the default selects the wrong sensor |
| `OP_CAMERA_FLIP` | `2` | wrong orientation otherwise |
| GPS | `ttyTHS1` @ **38400** | not the usual 9600 |
| BNO085 | `0x4B` on **i2c-7** | not the default address or bus |
| BNO085 driver | SparkFun library | Adafruit_BNO08x silently delivers zero events on this board |

## Pin map (STM32F407)

```
PA2 / PA3     USART2 — ST-Link VCP (legacy link, unused on USB)
PB8  RX       CAN1 ──► transceiver 1 ──► car main bus      (logical bus 0)
PB9  TX
PB5  RX       CAN2 ──► transceiver 2 ──► camera / LKAS bus (logical bus 2)
PB6  TX
PA11 / PA12   USB OTG FS — host link to the Jetson
```

Free for an SPI host link: **SPI2** on PB12 (NSS), PB13 (SCK), PB14 (MISO), PB15 (MOSI).
SPI1 on PA4–PA7 is also free. Both sides are 3.3 V, so no level shifting.

### bxCAN will not start on a sleeping bus

The CAN core needs 11 consecutive recessive bits to leave initialisation. With the engine
off the bus never supplies them, and turning the ignition on afterwards does **not**
retrigger init. A wedged core reports `TEC=0 REC=0 bus_off=0 last_error="No error"`, so
nothing looks broken. Forwarding runs inside the receive path, so a wedged core also stops
bridging camera ↔ car and the cluster raises a front camera fault.

**Start the car before launching the stack.** Only a `0xd8` board reset clears it.

## CAN bus

Bus 0 carries ~2870 frames/s across 23 distinct IDs. The ones that matter:

| id | name | rate | role |
|---|---|---|---|
| `0x243` | CAM_LKAS | 50–70 Hz | steering command (ours replaces the camera's) |
| `0x21b` | CRZ_INFO | 50 Hz | carries `ACCEL_CMD` — radar's, or ours under alpha long |
| `0x21c` | CRZ_CTRL | 50 Hz | cruise control state |
| `0x21f` | CRZ_EVENTS | 50 Hz | set speed; survives radar suppression |
| `0x165` | PEDALS | 100 Hz | `ACC_ACTIVE`, brake, gas |
| `0x09d` | CRZ_BTNS | 10 Hz | cruise buttons |
| `0x361`–`0x366`, `0x499` | radar tracks | 10 Hz | go silent under suppression |
| `0x764` / `0x76C` | radar UDS | — | request / response

The radar answers exactly one diagnostic service: `10 02`, enter programming session.
`19 02 FF`, `19 0A`, `10 03`, and OBD `03` / `07` all get no reply at all.

## Flashing

```bash
# bridge BOOT0 to 3V3, power-cycle, then:
dfu-util -d 0483:df11 -a 0 -s 0x08000000       -D board/obj/bootstub.panda_f407.bin
dfu-util -d 0483:df11 -a 0 -s 0x08004000:leave -D board/obj/panda_f407.bin.signed
```

Sector 0 (16K) holds the bootstub; the application starts at `0x08004000`.
