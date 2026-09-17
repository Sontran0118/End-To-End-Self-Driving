// Peripheral init for the Nucleo-F446RE panda port.
// Link to Jetson = USART2 over the ST-Link VCP (PA2=TX, PA3=RX). No native USB used.
// CAN1 = PB8(RX)/PB9(TX) [AF9] -> transceiver 1 -> car main bus
// CAN2 = PB5(RX=D4)/PB6(TX=D10) [AF9] -> transceiver 2 -> car camera/LKAS bus

void gpio_usart2_init(void) {
  // PA2/PA3: USART2 (ST-Link VCP). AF7.
  set_gpio_alternate(GPIOA, 2, GPIO_AF7_USART2);
  set_gpio_alternate(GPIOA, 3, GPIO_AF7_USART2);
}

void gpio_can_init(void) {
  // CAN1: PB8 RX, PB9 TX (AF9)
  set_gpio_alternate(GPIOB, 8, GPIO_AF9_CAN1);
  set_gpio_alternate(GPIOB, 9, GPIO_AF9_CAN1);
  // CAN2: PB5 RX (D4), PB6 TX (D10) (AF9) -> transceiver 2
  set_gpio_alternate(GPIOB, 5, GPIO_AF9_CAN2);
  set_gpio_alternate(GPIOB, 6, GPIO_AF9_CAN2);

#ifdef CAN_RX_BENCH_PULLUP
  /* BENCH DIAGNOSTIC ONLY -- DO NOT SHIP. NOT enabled by any build in SConscript.
   *
   * MEASURED 2026-08-07: both RX pins are high-impedance -- each follows the
   * MCU's internal pull-up to 1 and its pull-down to 0, so no transceiver is
   * driving either line. With PUPDR=0 a floating input sits wherever leakage
   * puts it; PB8 happened to rest low and PB5 near the threshold picking up
   * noise, which looked like two different faults and is one.
   *
   * bxCAN will not leave initialisation until it has seen 11 consecutive
   * RECESSIVE bits on RX, so a floating RX pin strands the core with
   * MSR.INAK=1 -- which is why CAN1 has no interrupts enabled and an inactive
   * filter bank. CAN2 escaped only because its floating pin bounced enough to
   * fake a qualifying run.
   *
   * Forcing the idle state recessive lets CAN1 initialise, so internal loopback
   * can prove the PERIPHERAL independently of the missing transceiver.
   *
   * Why this must never ship: 40k is trivially overridden by a real transceiver,
   * so it buys nothing when the hardware is right -- and when the hardware is
   * WRONG it makes a disconnected bus read as an idle one. That converts "my
   * transceiver fell off" into "the bus is quiet", hiding exactly the fault
   * that most needs to be loud in a car.
   */
  set_gpio_pullup(GPIOB, 8, PULL_UP);
  set_gpio_pullup(GPIOB, 5, PULL_UP);
#endif
}

// Common GPIO initialization
void common_init_gpio(void) {
  // GPIO clocks FIRST - touching a GPIO port with its clock gated hangs the bus.
  // (peripherals_init() enables these too, but it runs AFTER this function.)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  // short settling delay after enabling a peripheral clock
  for (volatile int _i = 0; _i < 32; _i++);

  GPIOA->ODR = 0;
  GPIOB->ODR = 0;
  GPIOA->PUPDR = 0;
  GPIOB->AFR[0] = 0;
  GPIOB->AFR[1] = 0;

  gpio_usart2_init();
  gpio_can_init();

  // Nucleo LD2 user LED on PA5 (status)
  set_gpio_output(GPIOA, 5, false);
}

// USART2 baud config. panda uses 115200 for debug; the serial link runs faster.
// For the VCP link we use 1.5 Mbaud to keep CAN throughput.
#ifndef SERIAL_UART
#define SERIAL_UART USART2
#endif
#define SERIAL_BAUD 1500000U

void usart2_init(void) {
  // 8N1, TX+RX enabled, oversampling by 8 for high baud
  SERIAL_UART->CR1 = USART_CR1_OVER8;
  // BRR for OVER8: USARTDIV = fCK / (8 * baud); fCK = APB1.
  // mantissa in [15:4], fraction (3 bits) in [2:0]
  //
  // Derive fCK from APB1_FREQ rather than hardcoding it. This was carried over
  // from the F446 layer as a literal 45000000, but the F407 runs APB1 at 42 MHz
  // (168/4, and 42 is the part's APB1 maximum). Computing the divisor for a
  // clock 7% faster than reality puts the real line rate at ~1.40 Mbaud instead
  // of 1.50 -- far outside the ~2-3% a UART tolerates, so every frame arrives
  // corrupted with no obvious clue as to why.
  {
    uint32_t fck = APB1_FREQ * 1000000U;
    uint32_t div8 = (fck * 2U) / SERIAL_BAUD;   // = 2*fck/baud (for OVER8 scaling)
    uint32_t mant = div8 / 16U;
    uint32_t frac = div8 & 0x0FU;               // 4-bit -> use low 3 for OVER8
    SERIAL_UART->BRR = (mant << 4) | ((frac >> 1) & 0x07U);
  }
  SERIAL_UART->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

// Peripheral initialization
// defined below; peripherals_init() needs it before its definition
void gpio_usb_init(void);

void peripherals_init(void) {
  // enable GPIO(A,B,C)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  // Connectivity: USART2 (VCP link) + CAN1 + CAN2
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
  RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;   // CAN2 shares CAN1's clock domain (master/slave bxCAN)

  // USB OTG FS. This used to live ONLY in flasher_peripherals_init(), which just
  // the bootstub's soft-flasher calls -- so in the main firmware the USB core was
  // never clocked. usb_init() opens with
  //     while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);
  // and an unclocked peripheral reads back as zero, so that spin never exits and
  // the firmware hangs before ever attaching to the bus. From the host it looks
  // like a board that is plugged in but electrically absent: no enumeration, and
  // not even a failed attach in dmesg.
  //
  // It stayed hidden because the F446 build uses the serial transport and never
  // calls usb_init() at all.
  RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;

  common_init_gpio();
  gpio_usb_init();   // PA11/PA12 -> AF10, must happen before usb_init()
  usart2_init();
}

// panda calls this to (re)bring the debug/comms uart. We route it to USART2.
void f446_uart_init(USART_TypeDef *u, int baud) {
  UNUSED(u); UNUSED(baud);
  usart2_init();
}

// --- ported from stm32f4/peripherals.h (F446-compatible) ---
void enable_interrupt_timer(void) {
  register_set_bits(&(RCC->APB1ENR), RCC_APB1ENR_TIM6EN);  // Enable interrupt timer peripheral
}

void flasher_peripherals_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
}

void gpio_spi_init(void) {
}

void gpio_usb_init(void) {
  // A11,A12: USB
  set_gpio_alternate(GPIOA, 11, GPIO_AF10_OTG_FS);
  set_gpio_alternate(GPIOA, 12, GPIO_AF10_OTG_FS);
  GPIOA->OSPEEDR = GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12;
}
