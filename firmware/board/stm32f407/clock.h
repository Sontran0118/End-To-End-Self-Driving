// STM32F407VET6 core board clock config (2026-08-05).
//
// Derived from the F446 Nucleo config, but the two parts differ in three ways
// that all matter here:
//
//   1. F407 tops out at 168 MHz, not 180. There is no overdrive mode
//      (PWR_CR_ODEN / ODSWEN do not exist), so that whole block is gone.
//   2. F407 has no PLLSAI and no DCKCFGR2/CK48MSEL. The 48 MHz USB/SDIO clock
//      must come from the MAIN PLL's Q output, which constrains the VCO: it has
//      to be a multiple of 48. 336 MHz works (336/7 = 48, 336/2 = 168 SYSCLK).
//   3. APB maximums are lower: APB1 42 MHz (not 45), APB2 84 MHz (not 90).
//
// HSE: these boards ship an 8 MHz crystal. VERIFY THE MARKING before flashing --
// if it is 25 MHz (some variants are), PLLM must be 25 instead of 8 or every
// derived clock is wrong by that ratio, and the first symptom is a CAN bus that
// looks dead rather than an error, because every bit time is off.
//
// From 8 MHz HSE: PLLM=8 -> 1 MHz ref; PLLN=336 -> 336 MHz VCO;
//                 PLLP=2 -> 168 MHz SYSCLK; PLLQ=7 -> 48 MHz USB.
// Buses: AHB=/1 (168), APB1=/4 (42, max), APB2=/2 (84, max).

void clock_init(void) {
  // Try the external crystal, but NEVER spin on it forever. A board whose HSE
  // does not start would hang here before USB is initialised, which from the
  // host looks identical to a failed flash -- no enumeration, no DFU, nothing
  // to talk to. Falling back to HSI keeps the device reachable so it can at
  // least be re-flashed.
  //
  // HSI is 16 MHz (vs 8 MHz HSE), so PLLM doubles to 16 to land on the same
  // 1 MHz PLL input, keeping PLLN/PLLP/PLLQ -- and therefore SYSCLK and the
  // 48 MHz USB clock -- identical either way.
  //
  // Caveat: HSI is only +-1%, while USB full-speed wants +-0.25%. On HSI the
  // USB link may enumerate unreliably or not at all. This is a recovery path,
  // not a supported operating mode -- if the board lands here, the crystal
  // needs fixing.
  bool hse_ok = false;
  register_set_bits(&(RCC->CR), RCC_CR_HSEON);
  for (uint32_t i = 0U; i < 0x200000U; i++) {
    if ((RCC->CR & RCC_CR_HSERDY) != 0U) {
      hse_ok = true;
      break;
    }
  }

  // enable power controller clock, voltage scale 1 (required above 144 MHz)
  register_set_bits(&(RCC->APB1ENR), RCC_APB1ENR_PWREN);
  register_set(&(PWR->CR), PWR_CR_VOS, PWR_CR_VOS);

  // bus prescalers: AHB=/1 (168), APB1=/4 (42, max 42), APB2=/2 (84, max 84)
  register_set(&(RCC->CFGR), RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2, 0xFF7FFCF3U);

  // main PLL: PLLN=336, PLLP=2 (=00), PLLQ=7 -> 168 MHz SYSCLK, 48 MHz USB.
  // PLLM/PLLSRC depend on which oscillator actually came up.
  register_set(&(RCC->PLLCFGR),
    ((hse_ok ? 8U : 16U) << RCC_PLLCFGR_PLLM_Pos) |
    (336U << RCC_PLLCFGR_PLLN_Pos) |
    (0U   << RCC_PLLCFGR_PLLP_Pos) |
    (7U   << RCC_PLLCFGR_PLLQ_Pos) |
    (hse_ok ? RCC_PLLCFGR_PLLSRC_HSE : RCC_PLLCFGR_PLLSRC_HSI), 0x0F437FFFU);

  // flash: 5 wait states for 168 MHz at 3.3 V (VOS1), prefetch and caches on
  register_set(&(FLASH->ACR), FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS, 0x1F0FU);

  // start main PLL
  register_set_bits(&(RCC->CR), RCC_CR_PLLON);
  while ((RCC->CR & RCC_CR_PLLRDY) == 0U);

  // switch SYSCLK to main PLL
  register_set_bits(&(RCC->CFGR), RCC_CFGR_SW_PLL);
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

  // *** running on 168 MHz PLL, USB/SDIO 48 MHz from PLLQ ***
}
