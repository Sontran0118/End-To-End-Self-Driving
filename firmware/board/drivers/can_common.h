#include "can_common_declarations.h"

uint32_t safety_tx_blocked = 0;
uint32_t safety_rx_invalid = 0;
uint32_t tx_buffer_overflow = 0;
uint32_t rx_buffer_overflow = 0;

can_health_t can_health[CAN_HEALTH_ARRAY_SIZE] = {{0}, {0}, {0}};

// Ignition detected from CAN meessages
bool ignition_can = false;
uint32_t ignition_can_cnt = 0U;

int can_live = 0;
int pending_can_live = 0;
int can_silent = ALL_CAN_SILENT;
bool can_loopback = false;

// ********************* instantiate queues *********************
#define can_buffer(x, size) \
  static CANPacket_t elems_##x[size]; \
  extern can_ring can_##x; \
  can_ring can_##x = { .w_ptr = 0, .r_ptr = 0, .fifo_size = (size), .elems = (CANPacket_t *)&(elems_##x) };

// F446 and F407 both have 128K SRAM, against the H7's much larger budget these
// defaults were written for. At 4096/416 the queues alone are ~80KB and .bss
// overflows the region by ~3.5KB; 2048/128 keeps ~20KB of stack headroom.
#if defined(STM32F446xx) || defined(STM32F407xx)
#define CAN_RX_BUFFER_SIZE 2048U   // deep queue, keeps ~20KB stack headroom
#else
#define CAN_RX_BUFFER_SIZE 4096U
#endif
#if defined(STM32F446xx) || defined(STM32F407xx)
#define CAN_TX_BUFFER_SIZE 128U
#else
#define CAN_TX_BUFFER_SIZE 416U
#endif

#ifdef STM32H7
// ITCM RAM and DTCM RAM are the fastest for Cortex-M7 core access
__attribute__((section(".axisram"))) can_buffer(rx_q, CAN_RX_BUFFER_SIZE)
__attribute__((section(".itcmram"))) can_buffer(tx1_q, CAN_TX_BUFFER_SIZE)
__attribute__((section(".itcmram"))) can_buffer(tx2_q, CAN_TX_BUFFER_SIZE)
#else
can_buffer(rx_q, CAN_RX_BUFFER_SIZE)
can_buffer(tx1_q, CAN_TX_BUFFER_SIZE)
can_buffer(tx2_q, CAN_TX_BUFFER_SIZE)
#endif
can_buffer(tx3_q, CAN_TX_BUFFER_SIZE)

// FIXME:
// cppcheck-suppress misra-c2012-9.3
can_ring *can_queues[CAN_QUEUES_ARRAY_SIZE] = {&can_tx1_q, &can_tx2_q, &can_tx3_q};

// ********************* interrupt safe queue *********************
bool can_pop(can_ring *q, CANPacket_t *elem) {
  bool ret = 0;

  ENTER_CRITICAL();
  if (q->w_ptr != q->r_ptr) {
    *elem = q->elems[q->r_ptr];
    if ((q->r_ptr + 1U) == q->fifo_size) {
      q->r_ptr = 0;
    } else {
      q->r_ptr += 1U;
    }
    ret = 1;
  }
  EXIT_CRITICAL();

  return ret;
}

bool can_push(can_ring *q, const CANPacket_t *elem) {
  bool ret = false;
  uint32_t next_w_ptr;

  ENTER_CRITICAL();
  if ((q->w_ptr + 1U) == q->fifo_size) {
    next_w_ptr = 0;
  } else {
    next_w_ptr = q->w_ptr + 1U;
  }
  if (next_w_ptr != q->r_ptr) {
    q->elems[q->w_ptr] = *elem;
    q->w_ptr = next_w_ptr;
    ret = true;
  }
  EXIT_CRITICAL();
  if (!ret) {
    #ifdef DEBUG
      print("can_push to ");
      if (q == &can_rx_q) {
        print("can_rx_q");
      } else if (q == &can_tx1_q) {
        print("can_tx1_q");
      } else if (q == &can_tx2_q) {
        print("can_tx2_q");
      } else if (q == &can_tx3_q) {
        print("can_tx3_q");
      } else {
        print("unknown");
      }
      print(" failed!\n");
    #endif
  }
  return ret;
}

uint32_t can_slots_empty(const can_ring *q) {
  uint32_t ret = 0;

  ENTER_CRITICAL();
  if (q->w_ptr >= q->r_ptr) {
    ret = q->fifo_size - 1U - q->w_ptr + q->r_ptr;
  } else {
    ret = q->r_ptr - q->w_ptr - 1U;
  }
  EXIT_CRITICAL();

  return ret;
}

void can_clear(can_ring *q) {
  ENTER_CRITICAL();
  q->w_ptr = 0;
  q->r_ptr = 0;
  EXIT_CRITICAL();
  // handle TX buffer full with zero ECUs awake on the bus
  refresh_can_tx_slots_available();
}

// assign CAN numbering
// bus num: CAN Bus numbers in panda, sent to/from USB
//    Min: 0; Max: 127; Bit 7 marks message as receipt (bus 129 is receipt for but 1)
// cans: Look up MCU can interface from bus number
// can number: numeric lookup for MCU CAN interfaces (0 = CAN1, 1 = CAN2, etc);
// bus_lookup: Translates from 'can number' to 'bus number'.
// can_num_lookup: Translates from 'bus number' to 'can number'.
// forwarding bus: If >= 0, forward all messages from this bus to the specified bus.

// Helpers
// Panda:       Bus 0=CAN1   Bus 1=CAN2   Bus 2=CAN3
#ifdef STM32F446xx
// F446 has only two CAN peripherals, but openpilot's forwarding convention is
// bus 0 <-> bus 2 (get_fwd_bus() in the opendbc safety layer): bus 0 = main/
// powertrain, bus 2 = camera/ADAS. Bus 1 never forwards. So physical CAN2 must
// present as LOGICAL BUS 2, not bus 1, or no frame is ever eligible to forward.
//
// bus_config is indexed two different ways, which is easy to get wrong:
//   .bus_lookup     is read as bus_config[can_number].bus_lookup     (can -> bus)
//   .can_num_lookup is read as bus_config[bus_number].can_num_lookup (bus -> can)
// So index 1 serves BOTH roles: as can_number 1 it maps to bus 2, and as
// bus_number 1 it maps to no CAN (0xFF) because nothing lives on bus 1 here.
// Index 2 is bus 2 -> can 1; its .bus_lookup is unused (there is no can_number 2).
bus_config_t bus_config[BUS_CONFIG_ARRAY_SIZE] = {
  { .bus_lookup = 0U, .can_num_lookup = 0U, .forwarding_bus = -1, .can_speed = 5000U, .can_data_speed = 20000U, .canfd_auto = false, .canfd_enabled = false, .brs_enabled = false, .canfd_non_iso = false },
  { .bus_lookup = 2U, .can_num_lookup = 0xFFU, .forwarding_bus = -1, .can_speed = 5000U, .can_data_speed = 20000U, .canfd_auto = false, .canfd_enabled = false, .brs_enabled = false, .canfd_non_iso = false },
  { .bus_lookup = 0xFFU, .can_num_lookup = 1U, .forwarding_bus = -1, .can_speed = 5000U, .can_data_speed = 20000U, .canfd_auto = false, .canfd_enabled = false, .brs_enabled = false, .canfd_non_iso = false },
  { .bus_lookup = 0xFFU, .can_num_lookup = 0xFFU, .forwarding_bus = -1, .can_speed = 333U, .can_data_speed = 333U, .canfd_auto = false, .canfd_enabled = false, .brs_enabled = false, .canfd_non_iso = false },
};
#else
bus_config_t bus_config[BUS_CONFIG_ARRAY_SIZE] = {
  { .bus_lookup = 0U, .can_num_lookup = 0U, .forwarding_bus = -1, .can_speed = 5000U, .can_data_speed = 20000U, .canfd_auto = false, .canfd_enabled = false, .brs_enabled = false, .canfd_non_iso = false },
  { .bus_lookup = 1U, .can_num_lookup = 1U, .forwarding_bus = -1, .can_speed = 5000U, .can_data_speed = 20000U, .canfd_auto = false, .canfd_enabled = false, .brs_enabled = false, .canfd_non_iso = false },
  { .bus_lookup = 2U, .can_num_lookup = 2U, .forwarding_bus = -1, .can_speed = 5000U, .can_data_speed = 20000U, .canfd_auto = false, .canfd_enabled = false, .brs_enabled = false, .canfd_non_iso = false },
  { .bus_lookup = 0xFFU, .can_num_lookup = 0xFFU, .forwarding_bus = -1, .can_speed = 333U, .can_data_speed = 333U, .canfd_auto = false, .canfd_enabled = false, .brs_enabled = false, .canfd_non_iso = false },
};
#endif

void can_init_all(void) {
  for (uint8_t i=0U; i < PANDA_CAN_CNT; i++) {
    #ifndef CANFD
      bus_config[i].can_data_speed = 0U;
    #endif
    can_clear(can_queues[i]);
#if defined(STM32F446xx) || defined(STM32F407xx)
    // F446 and F407 have only CAN1 (can 0) and CAN2 (can 1). There is no can_number 2:
    // cans[2] aliases CAN2, so can_init(2) would RE-initialize the already-running
    // CAN2 -- re-entering INRQ mid-operation and re-clocking the peripheral, leaving
    // it mis-synced so it STUFF/FORM-errors on real traffic. Skip the phantom init.
    if (i >= F446_CAN_CNT) { continue; }
#endif
    (void)can_init(i);
  }
}

void can_set_orientation(bool flipped) {
#ifdef STM32F446xx
  // The Nucleo has no harness relay / SBU orientation detection, and the stock
  // swap below assumes can_number 2 exists (it does not here) -- applying it
  // would overwrite the F446 can<->bus mapping set up above. Fixed orientation.
  UNUSED(flipped);
#else
  bus_config[0].bus_lookup = flipped ? 2U : 0U;
  bus_config[0].can_num_lookup = flipped ? 2U : 0U;
  bus_config[2].bus_lookup = flipped ? 0U : 2U;
  bus_config[2].can_num_lookup = flipped ? 0U : 2U;
#endif
}

#ifdef PANDA_JUNGLE
void can_set_forwarding(uint8_t from, uint8_t to) {
  bus_config[from].forwarding_bus = to;
}
#endif

void ignition_can_hook(CANPacket_t *msg) {
  int bus = GET_BUS(msg);
  if (bus == 0) {
    int addr = GET_ADDR(msg);
    int len = GET_LEN(msg);

    // GM exception
    if ((addr == 0x1F1) && (len == 8)) {
      // SystemPowerMode (2=Run, 3=Crank Request)
      ignition_can = (msg->data[0] & 0x2U) != 0U;
      ignition_can_cnt = 0U;
    }

    // Rivian R1S/T GEN1 exception
    if ((addr == 0x152) && (len == 8)) {
      // 0x152 overlaps with Subaru pre-global which has this bit as the high beam
      int counter = msg->data[1] & 0xFU;  // max is only 14

      static int prev_counter_rivian = -1;
      if ((counter == ((prev_counter_rivian + 1) % 15)) && (prev_counter_rivian != -1)) {
        // VDM_OutputSignals->VDM_EpasPowerMode
        ignition_can = ((msg->data[7] >> 4U) & 0x3U) == 1U;  // VDM_EpasPowerMode_Drive_On=1
        ignition_can_cnt = 0U;
      }
      prev_counter_rivian = counter;
    }

    // Tesla Model 3/Y exception
    if ((addr == 0x221) && (len == 8)) {
      // 0x221 overlaps with Rivian which has random data on byte 0
      int counter = msg->data[6] >> 4;

      static int prev_counter_tesla = -1;
      if ((counter == ((prev_counter_tesla + 1) % 16)) && (prev_counter_tesla != -1)) {
        // VCFRONT_LVPowerState->VCFRONT_vehiclePowerState
        int power_state = (msg->data[0] >> 5U) & 0x3U;
        ignition_can = power_state == 0x3;  // VEHICLE_POWER_STATE_DRIVE=3
        ignition_can_cnt = 0U;
      }
      prev_counter_tesla = counter;
    }

    // Mazda exception
    if ((addr == 0x9E) && (len == 8)) {
      ignition_can = (msg->data[0] >> 5) == 0x6U;
      ignition_can_cnt = 0U;
    }

#ifdef PANDA_NUCLEO
    // Mazda CX-5 2023 (MAZDA_CX5_2022): this platform does not transmit 0x9E
    // (MSG_05) at all, so the exception above never fires and ignition_can stays
    // false forever -- which makes pandad hold NO_OUTPUT permanently, since its
    // relay logic is `!ignition_local || !is_onroad`.
    //
    // Use ENGINE_DATA (0x202) instead: RPM is a big-endian 16-bit at bit 7 with a
    // 0.25 scale, so raw = (d[0]<<8)|d[1] and rpm = raw/4. A running engine idles
    // around 650-750 rpm; key-on-engine-off reads 0. Threshold well below idle but
    // above noise. 0x202 is sent at ~120 Hz, comfortably faster than the 2 s
    // ignition_can timeout in main.c.
    if ((addr == 0x202) && (len == 8)) {
      uint32_t rpm_raw = ((uint32_t)msg->data[0] << 8) | (uint32_t)msg->data[1];
      ignition_can = (rpm_raw / 4U) > MAZDA_IGNITION_MIN_RPM;
      ignition_can_cnt = 0U;
    }
#endif

  }
}

bool can_tx_check_min_slots_free(uint32_t min) {
  return
    (can_slots_empty(&can_tx1_q) >= min) &&
    (can_slots_empty(&can_tx2_q) >= min) &&
    (can_slots_empty(&can_tx3_q) >= min);
}

uint8_t calculate_checksum(const uint8_t *dat, uint32_t len) {
  uint8_t checksum = 0U;
  for (uint32_t i = 0U; i < len; i++) {
    checksum ^= dat[i];
  }
  return checksum;
}

void can_set_checksum(CANPacket_t *packet) {
  packet->checksum = 0U;
  packet->checksum = calculate_checksum((uint8_t *) packet, CANPACKET_HEAD_SIZE + GET_LEN(packet));
}

bool can_check_checksum(CANPacket_t *packet) {
  return (calculate_checksum((uint8_t *) packet, CANPACKET_HEAD_SIZE + GET_LEN(packet)) == 0U);
}

void can_send(CANPacket_t *to_push, uint8_t bus_number, bool skip_tx_hook) {
  if (skip_tx_hook || safety_tx_hook(to_push) != 0) {
    if (bus_number < PANDA_BUS_CNT) {
      // add CAN packet to send queue
      tx_buffer_overflow += can_push(can_queues[bus_number], to_push) ? 0U : 1U;
      process_can(CAN_NUM_FROM_BUS_NUM(bus_number));
    }
  } else {
    safety_tx_blocked += 1U;
    to_push->returned = 0U;
    to_push->rejected = 1U;

    // data changed
    can_set_checksum(to_push);
    rx_buffer_overflow += can_push(&can_rx_q, to_push) ? 0U : 1U;
  }
}

bool is_speed_valid(uint32_t speed, const uint32_t *all_speeds, uint8_t len) {
  bool ret = false;
  for (uint8_t i = 0U; i < len; i++) {
    if (all_speeds[i] == speed) {
      ret = true;
    }
  }
  return ret;
}
