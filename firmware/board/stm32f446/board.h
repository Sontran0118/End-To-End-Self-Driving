// ///////////////////////////////////////////////////////////// //
// Hardware abstraction layer - Nucleo-F446RE DIY panda          //
// ///////////////////////////////////////////////////////////// //
#include "board/boards/board_declarations.h"
#include "board/boards/unused_funcs.h"

// ///// Board definition and detection ///// //
#include "board/stm32f446/lladc.h"
#include "board/drivers/harness.h"
#include "board/drivers/fan.h"
#include "board/stm32f446/llfan.h"
#include "board/drivers/clock_source.h"
#include "board/boards/nucleo.h"

void detect_board_type(void) {
  hw_type = HW_TYPE_DOS;   // closest real F4 2-CAN type; UNKNOWN is rejected by main.c
  current_board = &board_nucleo;
}
