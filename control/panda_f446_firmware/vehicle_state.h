// Mazda CX-5 vehicle-state decoding, done on-device.
// Scalings from opendbc/dbc/mazda_2017.dbc. Motorola/big-endian signals.
#pragma once

typedef struct __attribute__((packed)) {
  uint16_t counter;        // increments per decoded frame set
  int16_t  steer_angle;    // deg * 10
  int16_t  steer_rate;     // deg/s * 10
  int16_t  steer_torque;   // motor torque * 10
  int8_t   torque_sensor;  // raw sensor, -127 offset applied
  uint16_t speed;          // kph * 100
  uint16_t rpm;            // rpm
  uint16_t pedal_gas;      // raw 12-bit
  uint16_t wheel_fl;       // kph * 100
  uint16_t wheel_fr;
  uint16_t wheel_rl;
  uint16_t wheel_rr;
  uint32_t rx_frames;      // frames decoded since boot
} VehicleState_t;

VehicleState_t vehicle_state = {0};

// Extract a Motorola/big-endian DBC signal: start_bit is the MSB position
// using the DBC convention (byte*8 + (7 - bit_in_byte) is NOT applied here;
// callers pass the raw DBC start bit and we walk MSB-first).
static uint32_t vs_sig(const uint8_t *d, uint8_t start_bit, uint8_t length) {
  uint32_t val = 0U;
  uint8_t byte_i = start_bit / 8U;
  uint8_t bit_i = start_bit % 8U;
  for (uint8_t i = 0U; i < length; i++) {
    if (byte_i >= 8U) { break; }
    val = (val << 1) | ((d[byte_i] >> bit_i) & 1U);
    if (bit_i == 0U) { bit_i = 7U; byte_i++; } else { bit_i--; }
  }
  return val;
}

static void vehicle_state_update(uint32_t addr, const uint8_t *d) {
  switch (addr) {
    case 0x082U:  // STEER: STEER_ANGLE 23|16 (0.05, -1600) deg
      vehicle_state.steer_angle = (int16_t)(((int32_t)vs_sig(d, 23U, 16U) * 5 / 10) - 16000);
      break;
    case 0x241U:  // STEER_RATE: 23|16 (0.25, -8192) deg/s
      vehicle_state.steer_rate = (int16_t)(((int32_t)vs_sig(d, 23U, 16U) * 25 / 10) - 81920);
      break;
    case 0x240U: { // STEER_TORQUE
      uint32_t raw = vs_sig(d, 46U, 15U);
      int32_t sgn = (raw & 0x4000U) ? ((int32_t)raw - 32768) : (int32_t)raw;
      vehicle_state.steer_torque = (int16_t)sgn;
      vehicle_state.torque_sensor = (int8_t)((int32_t)vs_sig(d, 7U, 8U) - 127);
      break;
    }
    case 0x202U:  // ENGINE_DATA: RPM 7|16 (0.25), SPEED 23|16 (0.01), PEDAL 39|12
      vehicle_state.rpm = (uint16_t)(vs_sig(d, 7U, 16U) / 4U);
      vehicle_state.speed = (uint16_t)vs_sig(d, 23U, 16U);
      vehicle_state.pedal_gas = (uint16_t)vs_sig(d, 39U, 12U);
      break;
    case 0x215U:  // WHEEL_SPEEDS: each 16 bit (0.01, -100) kph
      vehicle_state.wheel_fl = (uint16_t)vs_sig(d, 7U, 16U);
      vehicle_state.wheel_fr = (uint16_t)vs_sig(d, 23U, 16U);
      vehicle_state.wheel_rl = (uint16_t)vs_sig(d, 39U, 16U);
      vehicle_state.wheel_rr = (uint16_t)vs_sig(d, 55U, 16U);
      break;
    default:
      return;
  }
  vehicle_state.counter++;
  vehicle_state.rx_frames++;
}
