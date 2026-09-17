#pragma once

#include <stdbool.h>

//#define DEBUG
//#define DEBUG_UART
//#define DEBUG_USB
//#define DEBUG_SPI
//#define DEBUG_FAULTS
//#define DEBUG_COMMS
//#define DEBUG_FAN

#define CAN_INIT_TIMEOUT_MS 500U
#define USBPACKET_MAX_SIZE 0x40U
#define MAX_CAN_MSGS_PER_USB_BULK_TRANSFER 51U
#define MAX_CAN_MSGS_PER_SPI_BULK_TRANSFER 170U

// USB definitions
#define USB_VID 0x3801U

#ifdef PANDA_JUNGLE
  #ifdef BOOTSTUB
    #define USB_PID 0xDDEFU
  #else
    #define USB_PID 0xDDCFU
  #endif
#else
  #ifdef BOOTSTUB
    #define USB_PID 0xDDEEU
  #else
    #define USB_PID 0xDDCCU
  #endif
#endif

// platform includes
#ifdef STM32H7
  #include "board/stm32h7/stm32h7_config.h"
#elif defined(STM32F407xx)
  // F407VET6 core board. Its own layer rather than the generic stm32f4 one:
  // the clock tree differs (168 MHz, PLLQ for USB, no PLLSAI) and CORE_FREQ
  // feeds CAN bit timing, so sharing the 96 MHz generic config would put every
  // bit time out by 1.75x.
  #include "board/stm32f407/stm32f4_config.h"
#elif defined(STM32F446xx)
  #include "board/stm32f446/stm32f4_config.h"
#elif defined(STM32F4)
  #include "board/stm32f4/stm32f4_config.h"
#else
  // TODO: uncomment this, cppcheck complains
  // building for tests
  //#include "fake_stm.h"
#endif
