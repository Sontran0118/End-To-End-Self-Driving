#pragma once

// SAE 2284-3 : minimum 16 tq, SJW 3, sample point at 81.3%
//
// These numbers are per-MCU because they depend on APB1, and copying the F446's
// silently detunes the bus. The F446 runs APB1 at 45 MHz and uses 15 tq with
// brp=6 (45/(6*15) = 500 kbps exactly). The F407 runs APB1 at 42 MHz (168/4;
// 42 is this part's APB1 ceiling), where those same divisors give
// 42/(6*15) = 466.7 kbps -- 6.7% low. CAN needs bit timing inside a fraction of
// a percent, so every frame would fail, and the failure is silent: the board
// enumerates, reports no faults, and simply never exchanges a frame.
//
// 14 tq at 42 MHz restores exactness: 42/(6*14) = 500 kbps, and every entry in
// speeds[] (10/20/50/100/125/250/500/1000 kbps) divides with zero error too.
// Within 14 tq, SEQ1/SEQ2 = 10/3 puts the sample point at 78.6% -- inside the
// CiA 75-87.5% window and near the 81.3% the stock F413 uses -- while keeping
// SJW=3 (SJW must be <= SEQ2) for a 21% resync window. A wide SJW is what
// absorbs oscillator drift between nodes; a late sample with a narrow SJW shows
// up as FORM errors on a busy bus once this node starts ACKing (a silent
// listener is far more forgiving than an ACKing participant).
#define CAN_QUANTA 14U   // must equal 1+SEQ1+SEQ2
#define CAN_SEQ1 10U
#define CAN_SEQ2 3U   // F407: 14 tq @42MHz -> exactly 500 kbps (78.6% sample)
#define CAN_SJW  3U   // <= SEQ2

#define CAN_PCLK 42000U   // F407: APB1 = 168/4 = 42 MHz (F446 was 45, F413 48)
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
