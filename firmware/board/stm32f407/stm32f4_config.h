// Inherited from the F446 layer this was copied from, where it force-defined
// STM32F446xx. Left in place it defines BOTH variants, and stm32f4xx_hal_gpio_ex.h
// then activates two GPIO_GET_INDEX blocks (F405/F415/F407/F417 and
// F446/F412/F413/F423) -> "redefined" under -Werror.
#ifndef STM32F407xx
#define STM32F407xx
#endif
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio_ex.h"
#define MCU_IDCODE 0x413U   // STM32F405/407/415/417 (F446 was 0x421, F413 0x463)

#define CORE_FREQ 168U // in MHz - see stm32f407/clock.h (PLLM=8,PLLN=336,PLLP=2). F407 max is 168.
#define APB1_FREQ (CORE_FREQ/4U)        // PPRE1 = /4 -> 42 MHz (APB1 max on F407)
#define APB1_TIMER_FREQ (APB1_FREQ*2U)  // APB1 is multiplied by 2 for the timer peripherals
#define APB2_FREQ (CORE_FREQ/2U)        // PPRE2 = /2 -> 84 MHz (APB2 max on F407)
#define APB2_TIMER_FREQ (APB2_FREQ*2U)  // APB2 is multiplied by 2 for the timer peripherals

#define BOOTLOADER_ADDRESS 0x1FFF0004U

// Around (1Mbps / 8 bits/byte / 12 bytes per message)
#define CAN_INTERRUPT_RATE 60000U   // F446+Mazda: filtered bus bursts well past the F413-era 12000

#define MAX_LED_FADE 8192U

#define NUM_INTERRUPTS 102U                // There are 102 external interrupt sources (see stm32f413.h)

#define TICK_TIMER_IRQ TIM1_BRK_TIM9_IRQn
#define TICK_TIMER TIM9

#define MICROSECOND_TIMER TIM2

#define INTERRUPT_TIMER_IRQ TIM6_DAC_IRQn
#define INTERRUPT_TIMER TIM6

#define IND_WDG IWDG

#define PROVISION_CHUNK_ADDRESS 0x1FFF79E0U
#define DEVICE_SERIAL_NUMBER_ADDRESS 0x1FFF79C0U

#include "board/can.h"
#include "board/comms_definitions.h"

#ifndef BOOTSTUB
  #include "board/main_definitions.h"
#else
  #include "board/bootstub_declarations.h"
#endif

#include "board/libc.h"
#include "board/critical.h"
#include "board/faults.h"
#include "board/utils.h"

#include "board/drivers/registers.h"
#include "board/drivers/interrupts.h"
#include "board/drivers/gpio.h"
#include "board/stm32f407/peripherals.h"
#include "board/stm32f407/interrupt_handlers.h"
#include "board/drivers/timers.h"
#include "board/stm32f407/board.h"
#include "board/stm32f407/clock.h"

#if !defined(BOOTSTUB)
  #include "board/drivers/uart.h"
  #include "board/stm32f407/lluart.h"
#endif

#ifdef BOOTSTUB
  #include "board/stm32f407/llflash.h"
#else
  #include "board/stm32f407/llbxcan.h"
#endif

#include "board/stm32f407/llusb.h"

// unused
void spi_init(void) {};
void sound_tick(void) {};
void can_tx_comms_resume_spi(void) {};

void early_gpio_float(void) {
  RCC->AHB1ENR = RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

  GPIOB->MODER = 0; GPIOC->MODER = 0;
  GPIOA->ODR = 0; GPIOB->ODR = 0; GPIOC->ODR = 0;
  GPIOA->PUPDR = 0; GPIOB->PUPDR = 0; GPIOC->PUPDR = 0;
}
