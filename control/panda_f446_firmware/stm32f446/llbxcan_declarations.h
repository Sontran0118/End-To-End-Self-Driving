#pragma once

// SAE 2284-3 : minimum 16 tq, SJW 3, sample point at 81.3%
// F446 runs APB1 at 45 MHz (not the F413's 48), so 16 tq cannot divide evenly.
// 15 tq does: 45 MHz / (6 * 15) = exactly 500 kbps, and every other speed in
// speeds[] lands exact too. Within 15 tq, split SEQ1/SEQ2 as 11/3 rather than
// 12/2: that puts the sample point at 80.0% (vs 86.7%) -- much closer to the
// 81.3% the stock F413 uses and well inside the CiA 75-87.5% window -- and
// allows SJW=3, tripling... (SJW must be <= SEQ2) the resync window from 13.3%
// to 20% of a bit. A wide SJW is what absorbs oscillator drift between nodes;
// at 86.7%/SJW=2 the sample sat late in the bit with little room to resync,
// which showed up as FORM errors on the busy main bus once the board started
// ACKing (an ACKing node must resync tightly; a silent listener is forgiving).
#define CAN_QUANTA 15U   // must equal 1+SEQ1+SEQ2
#define CAN_SEQ1 11U
#define CAN_SEQ2 3U   // F446: 15 tq @45MHz -> exactly 500 kbps (80.0% sample)
#define CAN_SJW  3U   // <= SEQ2

#define CAN_PCLK 45000U   // F446: APB1 = 180/4 = 45 MHz (F413 was 48)
// 333 = 33.3 kbps
// 5000 = 500 kbps
#define can_speed_to_prescaler(x) (CAN_PCLK / CAN_QUANTA * 10U / (x))

#define CAN_NAME_FROM_CANIF(CAN_DEV) (((CAN_DEV)==CAN1) ? "CAN1" : (((CAN_DEV) == CAN2) ? "CAN2" : "CAN3"))

void print(const char *a);

// kbps multiplied by 10
#define SPEEDS_ARRAY_SIZE 8
extern const uint32_t speeds[SPEEDS_ARRAY_SIZE];
#define DATA_SPEEDS_ARRAY_SIZE 1
extern const uint32_t data_speeds[DATA_SPEEDS_ARRAY_SIZE]; // No separate data speed, dummy

bool llcan_set_speed(CAN_TypeDef *CANx, uint32_t speed, bool loopback, bool silent);
void llcan_irq_disable(const CAN_TypeDef *CANx);
void llcan_irq_enable(const CAN_TypeDef *CANx);
bool llcan_init(CAN_TypeDef *CANx);
void llcan_clear_send(CAN_TypeDef *CANx);
