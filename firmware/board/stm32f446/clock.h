// STM32F446RE clock config for the Nucleo panda port.
// Nucleo-F446RE default HSE source = 8 MHz (from ST-Link MCO on PH0/PF0, SB configured).
// If using an external 8 MHz crystal instead, same PLL values apply.
//
// Targets: SYSCLK = 180 MHz (max), AHB = 180, APB1 = 45, APB2 = 90.
// USB needs exactly 48 MHz: on F446 the main PLLQ can't give 48 from a 180 MHz VCO cleanly,
// so we drive the 48 MHz clock (USB/SDIO) from the dedicated PLLSAI-P output (CK48MSEL=1).
//
// Main PLL (from 8 MHz HSE): PLLM=8 -> 1 MHz ref; PLLN=360 -> 360 MHz VCO; PLLP=2 -> 180 MHz SYSCLK.
// PLLSAI (from same 8 MHz HSE, PLLM shared=8 -> 1 MHz): PLLSAIN=192 -> 192 MHz VCO; PLLSAIP=4 -> 48 MHz.

void clock_init(void) {
  // enable external oscillator (HSE)
  register_set_bits(&(RCC->CR), RCC_CR_HSEON);
  while ((RCC->CR & RCC_CR_HSERDY) == 0U);

  // enable power controller clock, set voltage scale 1 for 180 MHz
  register_set_bits(&(RCC->APB1ENR), RCC_APB1ENR_PWREN);
  register_set(&(PWR->CR), PWR_CR_VOS, PWR_CR_VOS);   // VOS = scale 1

  // bus prescalers: AHB=/1 (180), APB1=/4 (45, max 45), APB2=/2 (90, max 90)
  register_set(&(RCC->CFGR), RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2, 0xFF7FFCF3U);

  // main PLL: PLLM=8, PLLN=360, PLLP=2 (=00), PLLSRC=HSE. -> 180 MHz
  register_set(&(RCC->PLLCFGR),
    (8U  << RCC_PLLCFGR_PLLM_Pos) |
    (360U << RCC_PLLCFGR_PLLN_Pos) |
    (0U  << RCC_PLLCFGR_PLLP_Pos) |
    RCC_PLLCFGR_PLLSRC_HSE, 0x0F437FFFU);

  // enable overdrive for 180 MHz
  register_set_bits(&(PWR->CR), PWR_CR_ODEN);
  while ((PWR->CSR & PWR_CSR_ODRDY) == 0U);
  register_set_bits(&(PWR->CR), PWR_CR_ODSWEN);
  while ((PWR->CSR & PWR_CSR_ODSWRDY) == 0U);

  // flash: 5 wait states for 180 MHz @ VOS1, prefetch/caches on
  register_set(&(FLASH->ACR), FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS, 0x1F0FU);

  // start main PLL
  register_set_bits(&(RCC->CR), RCC_CR_PLLON);
  while ((RCC->CR & RCC_CR_PLLRDY) == 0U);

  // PLLSAI for the 48 MHz USB clock: PLLSAIN=192 (->192 MHz VCO), PLLSAIP=4 (->48 MHz).
  register_set(&(RCC->PLLSAICFGR),
    (192U << RCC_PLLSAICFGR_PLLSAIN_Pos) |
    (1U   << RCC_PLLSAICFGR_PLLSAIP_Pos), 0x7FFFFFFFU);   // PLLSAIP field: 1 == /4
  register_set_bits(&(RCC->CR), RCC_CR_PLLSAION);
  while ((RCC->CR & RCC_CR_PLLSAIRDY) == 0U);
  // select PLLSAI-P as the 48 MHz clock source
  register_set_bits(&(RCC->DCKCFGR2), RCC_DCKCFGR2_CK48MSEL);

  // switch SYSCLK to main PLL
  register_set_bits(&(RCC->CFGR), RCC_CFGR_SW_PLL);
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

  // *** running on 180 MHz PLL, USB on PLLSAI 48 MHz ***
}
