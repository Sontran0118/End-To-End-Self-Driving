// USB <-> UART passthrough, turning an Arduino Nano ESP32 into the USB-serial
// adapter this setup otherwise lacks.
//
// Two different jobs, hence MODE below:
//
//   MODE_PANDA_LINK  - the normal one. The F407 panda firmware is built with
//                      -DPANDA_NUCLEO, which never calls usb_init(); the panda
//                      protocol runs over USART2 instead (see board/main.c:339
//                      "no usable USB peripheral"). So the Jetson reaches the
//                      panda through this bridge, not through USB.
//                      1.5 Mbaud, 8N1, on the F407's PA2/PA3.
//
//   MODE_BOOTLOADER  - for talking to the STM32 ROM bootloader on USART1
//                      (PA9/PA10) at 115200 8E1. Kept because it is the
//                      fallback if DFU is ever unavailable. Note the ROM
//                      bootloader requires EVEN parity; 8N1 will never sync.
//
// NAMING GOTCHA: on the Nano ESP32 the physical D0/D1 header pins are Serial0,
// NOT Serial1. Serial1 is unassigned by default and would connect to nothing.
// `Serial` is the native USB CDC to the host.
//
// WIRING (always crossed over -- each TX goes to the other side's RX):
//
//   MODE_PANDA_LINK                     MODE_BOOTLOADER
//     Nano D1 (TX) -> F407 PA3 (RX)       Nano D1 (TX) -> F407 PA10 (RX, hdr pin 8)
//     Nano D0 (RX) -> F407 PA2 (TX)       Nano D0 (RX) -> F407 PA9  (TX, hdr pin 6)
//     Nano GND     -> F407 GND            Nano GND     -> F407 GND  (hdr pin 3)
//
// Header pin numbers are from the FK407M1 4x2 SWD/Serial header:
//   odd row  1=+5V  3=GND   5=PA13(DIO) 7=PA14(CLK)
//   even row 2=+3V3 4=NRST  6=PA9(TX)   8=PA10(RX)

#define MODE_PANDA_LINK  1
#define MODE_BOOTLOADER  2

#define MODE MODE_PANDA_LINK

#if MODE == MODE_PANDA_LINK
  static const uint32_t BAUD   = 1500000;   // must match SERIAL_BAUD in board/stm32f407/peripherals.h
  static const uint32_t CONFIG = SERIAL_8N1;
#else
  static const uint32_t BAUD   = 115200;
  static const uint32_t CONFIG = SERIAL_8E1; // ROM bootloader: even parity, non-negotiable
#endif

// At 1.5 Mbaud a byte lands every ~6.7 us, so the stock 256-byte driver buffer
// overruns during any scheduling hiccup on the USB side. Oversize both the
// driver ring and the copy buffer; dropped bytes here surface as CAN frames
// silently going missing, which is painful to debug from the openpilot end.
static const size_t RX_RING = 8192;
static uint8_t buf[1024];

void setup() {
  Serial.begin(BAUD);                 // baud ignored on native USB CDC
  Serial0.setRxBufferSize(RX_RING);   // must precede begin() to take effect
  Serial0.begin(BAUD, CONFIG);
}

void loop() {
  // Bulk moves rather than byte-at-a-time: keeps up without stalling either side.
  int n = Serial.available();
  if (n > 0) {
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    n = Serial.readBytes(buf, n);
    Serial0.write(buf, n);
  }
  n = Serial0.available();
  if (n > 0) {
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    n = Serial0.readBytes(buf, n);
    Serial.write(buf, n);
  }
}
