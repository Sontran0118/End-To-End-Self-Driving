// Board definition: STM32F446RE Nucleo panda (DIY panda replica for openpilot).
// 2 CAN buses (CAN1 PB8/PB9, CAN2 PB5/PB6) via 2 external transceivers.
// Link to Jetson over USART2 / ST-Link VCP. Minimal: no fan/siren/IR/harness-relay.
#include "board_declarations.h"

// Transceiver enable: most bare CAN transceiver modules (e.g. SN65HVD230/TJA1050) have no
// software enable, or tie STBY low = always-on. If your transceivers expose an enable/STBY pin,
// wire it and drive it here. Default: no-op (always enabled).
static void nucleo_enable_can_transceiver(uint8_t transceiver, bool enabled) {
  UNUSED(transceiver); UNUSED(enabled);
  // Example if you wire STBY of transceiver 1 to PC10, transceiver 2 to PC11:
  //   if (transceiver == 1U) { set_gpio_output(GPIOC, 10, !enabled); }
  //   if (transceiver == 2U) { set_gpio_output(GPIOC, 11, !enabled); }
}

static void nucleo_set_can_mode(uint8_t mode) {
  // Only NORMAL mode supported (no OBD relay on the Nucleo).
  UNUSED(mode);
}

static uint32_t nucleo_read_voltage_mV(void) { return 12000U; }   // no voltage sense; report nominal
static uint32_t nucleo_read_current_mA(void) { return 0U; }
static void nucleo_set_ir_power(uint8_t percentage) { UNUSED(percentage); }
static void nucleo_set_fan_enabled(bool enabled) { UNUSED(enabled); }
static void nucleo_set_siren(bool enabled) { UNUSED(enabled); }
static void nucleo_set_bootkick(BootState state) { UNUSED(state); }
static bool nucleo_read_som_gpio(void) { return false; }
static void nucleo_set_amp_enabled(bool enabled) { UNUSED(enabled); }

static void nucleo_init(void) {
  common_init_gpio();
  // bring up both CAN buses in normal mode
  nucleo_enable_can_transceiver(1U, true);
  nucleo_enable_can_transceiver(2U, true);
  // LD2 on to indicate alive
  set_gpio_output(GPIOA, 5, true);
}

static void nucleo_init_bootloader(void) { }

// no harness relay on the Nucleo
static harness_configuration nucleo_harness_config = {
  .GPIO_SBU1 = GPIOC,
  .GPIO_SBU2 = GPIOC,
  .GPIO_relay_SBU1 = GPIOC,
  .GPIO_relay_SBU2 = GPIOC,
  .pin_SBU1 = 0U,
  .pin_SBU2 = 1U,
  .pin_relay_SBU1 = 10U,
  .pin_relay_SBU2 = 11U,
  .adc_signal_SBU1 = ADC_CHANNEL_DEFAULT(ADC1, 10),
  .adc_signal_SBU2 = ADC_CHANNEL_DEFAULT(ADC1, 11),
};

struct board board_nucleo = {
  .harness_config = &nucleo_harness_config,
  // Only one usable LED on either board, but it is not the same pin:
  //   Nucleo-F446RE : LD2 = PA5
  //   FK407M1 F407  : user LED = PC13 (sink mode, i.e. lit when driven low)
  // Getting this wrong is silent -- led_set() just writes to a bare header pin --
  // which costs you the only sign of life a board with no working link can give.
#ifdef STM32F407xx
  .led_GPIO = {GPIOC, GPIOC, GPIOC},
  .led_pin = {13, 13, 13},
#else
  .led_GPIO = {GPIOA, GPIOA, GPIOA},
  .led_pin = {5, 5, 5},
#endif
  .led_pwm_channels = {0, 0, 0},
  .has_spi = false,
  .fan_max_rpm = 0U,
  .avdd_mV = 3300U,
  .fan_stall_recovery = false,
  .fan_enable_cooldown_time = 0U,
  .fan_max_pwm = 0U,
  .init = nucleo_init,
  .init_bootloader = nucleo_init_bootloader,
  .enable_can_transceiver = nucleo_enable_can_transceiver,
  .set_can_mode = nucleo_set_can_mode,
  .read_voltage_mV = nucleo_read_voltage_mV,
  .read_current_mA = nucleo_read_current_mA,
  .set_ir_power = nucleo_set_ir_power,
  .set_fan_enabled = nucleo_set_fan_enabled,
  .set_siren = nucleo_set_siren,
  .set_bootkick = nucleo_set_bootkick,
  .read_som_gpio = nucleo_read_som_gpio,
  .set_amp_enabled = nucleo_set_amp_enabled,
};
