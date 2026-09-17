#include "llbxcan_declarations.h"

// kbps multiplied by 10
const uint32_t speeds[SPEEDS_ARRAY_SIZE] = {100U, 200U, 500U, 1000U, 1250U, 2500U, 5000U, 10000U};
const uint32_t data_speeds[DATA_SPEEDS_ARRAY_SIZE] = {0U}; // No separate data speed, dummy

bool llcan_set_speed(CAN_TypeDef *CANx, uint32_t speed, bool loopback, bool silent) {
  bool ret = true;

  // wake the peripheral: bxCAN resets into SLEEP mode and will not receive
  // anything while SLAK is set.
  register_clear_bits(&(CANx->MCR), CAN_MCR_SLEEP);
  {
    uint32_t wake_to = 0U;
    while (((CANx->MSR & CAN_MSR_SLAK) == CAN_MSR_SLAK) && (wake_to < CAN_INIT_TIMEOUT_MS)) {
      delay(10000);
      wake_to++;
    }
  }

  // initialization mode
  register_set(&(CANx->MCR), CAN_MCR_TTCM | CAN_MCR_INRQ, 0x180FFU);
  uint32_t timeout_counter = 0U;
  while((CANx->MSR & CAN_MSR_INAK) != CAN_MSR_INAK){
    // Delay for about 1ms
    delay(10000);
    timeout_counter++;

    if(timeout_counter >= CAN_INIT_TIMEOUT_MS){
      print(CAN_NAME_FROM_CANIF(CANx)); print(" set_speed timed out (1)!\n");
      ret = false;
      break;
    }
  }

  if(ret){
    // set time quanta from defines
    register_set(&(CANx->BTR), ((CAN_BTR_TS1_0 * (CAN_SEQ1-1U)) |
                                   (CAN_BTR_TS2_0 * (CAN_SEQ2-1U)) |
                                   (CAN_BTR_SJW_0 * (CAN_SJW-1U)) |
                                   (can_speed_to_prescaler(speed) - 1U)), 0xC37F03FFU);

    // silent loopback mode for debugging
    if (loopback) {
      register_set_bits(&(CANx->BTR), CAN_BTR_SILM | CAN_BTR_LBKM);
    }
    if (silent) {
      register_set_bits(&(CANx->BTR), CAN_BTR_SILM);
    }

    // leave init mode; keep SLEEP clear (ABOM = auto bus-off recovery)
    register_set(&(CANx->MCR), CAN_MCR_TTCM | CAN_MCR_ABOM, 0x180FFU);

    timeout_counter = 0U;
    while(((CANx->MSR & CAN_MSR_INAK) == CAN_MSR_INAK)) {
      // Delay for about 1ms
      delay(10000);
      timeout_counter++;

      if(timeout_counter >= CAN_INIT_TIMEOUT_MS){
        print(CAN_NAME_FROM_CANIF(CANx)); print(" set_speed timed out (2)!\n");
        ret = false;
        break;
      }
    }
  }

  return ret;
}

void llcan_irq_disable(const CAN_TypeDef *CANx) {
  if (CANx == CAN1) {
    NVIC_DisableIRQ(CAN1_TX_IRQn);
    NVIC_DisableIRQ(CAN1_RX0_IRQn);
    NVIC_DisableIRQ(CAN1_SCE_IRQn);
  } else if (CANx == CAN2) {
    NVIC_DisableIRQ(CAN2_TX_IRQn);
    NVIC_DisableIRQ(CAN2_RX0_IRQn);
    NVIC_DisableIRQ(CAN2_SCE_IRQn);
  } else {
  }
}

void llcan_irq_enable(const CAN_TypeDef *CANx) {
  if (CANx == CAN1) {
    NVIC_EnableIRQ(CAN1_TX_IRQn);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    NVIC_EnableIRQ(CAN1_SCE_IRQn);
  } else if (CANx == CAN2) {
    NVIC_EnableIRQ(CAN2_TX_IRQn);
    NVIC_EnableIRQ(CAN2_RX0_IRQn);
    NVIC_EnableIRQ(CAN2_SCE_IRQn);
  } else {
  }
}

bool llcan_init(CAN_TypeDef *CANx) {
  bool ret = true;

  // Enter init mode
  register_set_bits(&(CANx->FMR), CAN_FMR_FINIT);

  // Wait for INAK bit to be set
  uint32_t timeout_counter = 0U;
  while(((CANx->MSR & CAN_MSR_INAK) == CAN_MSR_INAK)) {
    // Delay for about 1ms
    delay(10000);
    timeout_counter++;

    if(timeout_counter >= CAN_INIT_TIMEOUT_MS){
      print(CAN_NAME_FROM_CANIF(CANx)); print(" initialization timed out!\n");
      ret = false;
      break;
    }
  }

  if(ret){
#ifdef MAZDA_FILTER
    // ---- Hardware acceptance filter: ACCEPT EVERYTHING, both buses ----
    //
    // This used to be a 17-ID Mazda whitelist, written into both bank ranges, to
    // stop the ~743-ID / ~2-3k frames-per-second main bus from saturating the
    // 1.5 Mbaud serial link to the Jetson (observed 1.29M can_rx_q drops).
    //
    // That is the wrong layer. The acceptance filter sits UPSTREAM of can_rx(),
    // and can_rx() is where safety_fwd_hook() forwards cam<->car -- so a frame
    // dropped here is dropped from the FORWARD path too, not just the host feed.
    // The whitelist was therefore deleting, in silicon:
    //   cam -> car : 9 of the camera's 11 IDs (everything but 0x243 and 0x440 --
    //                CAM_LANETRACK, CAM_DISTANCE, CAM_PEDESTRIAN, CAM_TRAFFIC_SIGNS,
    //                CAM_SETTINGS, ...), which raises the front-camera fault.
    //   car -> cam : ~726 of the car's IDs, including every RADAR_* frame. A
    //                forward camera that used to see the whole main bus saw 17
    //                messages and no radar at all.
    // Stock panda forwards the entire bus in both directions; so do we now.
    //
    // The host link still cannot take the raw bus, so that trim moved into
    // software: mazda_host_visible() in drivers/mazda_filter.h, applied at the
    // can_rx_q push (and the tx echo) in drivers/bxcan.h. Same 17 IDs reach the
    // Jetson as before; the car and the camera now get everything.
    //
    // MASTER/SLAVE (F446): CAN1+CAN2 SHARE one filter block; CAN2 (slave) has NO
    // filter registers of its own -- ALL filter access must go through CAN1, and
    // the CAN2SB field in CAN1->FMR splits the banks. Writing CANx->sFilterRegister
    // for CAN2 silently does nothing, which is what once left CAN2 accepting no
    // frames at all. CAN2SB=14 -> bank 0 belongs to CAN1, bank 14 to CAN2.
    //
    // One 32-bit MASK-mode bank per bus, id=0 mask=0 -> matches every frame, both
    // standard and extended, data and remote.
    const uint8_t bank = (CANx == CAN1) ? 0U : 14U;
    const uint32_t bankbit = 1UL << bank;

    // filter config lives in the master (CAN1). enter filter-init on the master.
    register_set_bits(&(CAN1->FMR), CAN_FMR_FINIT);
    register_set(&(CAN1->FMR), (14UL << CAN_FMR_CAN2SB_Pos) | CAN_FMR_FINIT,
                 CAN_FMR_CAN2SB | CAN_FMR_FINIT);      // CAN2 start bank = 14

    CAN1->FA1R &= ~(0x1FUL << bank);                   // drop this bus's old banks
    CAN1->FS1R |= bankbit;                             // 32-bit scale
    CAN1->FM1R &= ~bankbit;                            // mask mode (not ID list)
    CAN1->FFA1R &= ~bankbit;                           // -> RX FIFO0, the one can_rx() reads
    CAN1->sFilterRegister[bank].FR1 = 0U;              // id   = 0
    CAN1->sFilterRegister[bank].FR2 = 0U;              // mask = 0 -> don't care
    CAN1->FA1R |= bankbit;                             // activate
    register_clear_bits(&(CAN1->FMR), CAN_FMR_FINIT);  // leave filter-init on master
#else
    // no mask
    // For some weird reason some of these registers do not want to set properly on CAN2 and CAN3. Probably something to do with the single/dual mode and their different filters.
    CANx->sFilterRegister[0].FR1 = 0U;
    CANx->sFilterRegister[0].FR2 = 0U;
    CANx->sFilterRegister[14].FR1 = 0U;
    CANx->sFilterRegister[14].FR2 = 0U;
    CANx->FA1R |= 1U | (1UL << 14);
#endif

    // Exit init mode, do not wait
    register_clear_bits(&(CANx->FMR), CAN_FMR_FINIT);

    // enable certain CAN interrupts
    register_set_bits(&(CANx->IER), CAN_IER_TMEIE | CAN_IER_FMPIE0 | CAN_IER_ERRIE | CAN_IER_LECIE | CAN_IER_BOFIE | CAN_IER_EPVIE | CAN_IER_EWGIE | CAN_IER_FOVIE0 | CAN_IER_FFIE0);

    // clear overrun flag on init
    CANx->RF0R &= ~(CAN_RF0R_FOVR0);

    llcan_irq_enable(CANx);
  }
  return ret;
}

void llcan_clear_send(CAN_TypeDef *CANx) {
  CANx->TSR |= CAN_TSR_ABRQ0; // Abort message transmission on error interrupt
  CANx->MSR |= CAN_MSR_ERRI; // Clear error interrupt
}
