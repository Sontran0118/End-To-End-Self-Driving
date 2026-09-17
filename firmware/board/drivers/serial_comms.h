// UART transport for the panda protocol (Nucleo-F446 over ST-Link VCP serial).
// Faithful port of the SPI transport framing (board/drivers/spi.h): same 6-byte header,
// same endpoints (0=control, 1/0x81=can_read, 2=ep2_write, 3=can_write), same XOR checksum,
// same comms_* dispatch — only the transport is UART instead of SPI DMA.
//
// Framing (host<->device):
//   HEADER  = [SYNC=0x5A][endpoint][mosi_len:u16 LE][miso_len:u16 LE][checksum:1]  (7 bytes)
//   host sends header; device replies 1 byte HACK(0x79)/NACK(0x1F).
//   if endpoint carries mosi data: host then sends [mosi_len data][data_checksum:1];
//     device replies [HACK][resp_len:u16][resp data][resp_checksum].
// Uses the same constants as spi.h so pandad's serial handle can share them.

#define SERIAL_SYNC_BYTE   0x5AU
#define SERIAL_HACK        0x79U
#define SERIAL_NACK        0x1FU
#define SERIAL_CHECKSUM_START 0xABU
#define SERIAL_HEADER_SIZE 7U
#define SERIAL_BUF_SIZE    2048U
// response framing overhead in ser_tx: [HACK][len:2][...data...][checksum]
#define SERIAL_RESP_OVERHEAD 4U

static uint8_t ser_rx[SERIAL_BUF_SIZE];
static uint8_t ser_tx[SERIAL_BUF_SIZE];

static uint8_t serial_checksum(const uint8_t *data, uint16_t len) {
  uint8_t c = SERIAL_CHECKSUM_START;
  for (uint16_t i = 0; i < len; i++) { c ^= data[i]; }
  return c;
}

// blocking UART helpers (defined by the ll uart layer: uart_send_byte / uart_recv_byte)
extern void uart_send_raw(const uint8_t *d, uint16_t len);
extern bool uart_recv_raw(uint8_t *d, uint16_t len, uint32_t timeout_ms);

// serial_stats itself is defined in serial_uart_raw.h -- the byte layer needs to
// bump rx_late, and it is included ahead of this file.

// process exactly one request/response transaction. Called from the main loop.
void serial_comms_tick(void) {
  // 0) genuine overrun is the ONLY case where discarding buffered bytes is right.
  //    The ring holds 27 ms at 1.5 Mbaud; if we are three quarters behind, the
  //    DMA is about to lap us and everything buffered is already suspect.
  if (uart_rx_avail() > ((UART_RX_BUF_SIZE * 3U) / 4U)) {
    serial_stats.overrun += 1U;
    uart_flush_rx();
    return;
  }

  // 1) hunt for the SYNC byte one byte at a time. Never consume a fixed block
  //    before we are aligned, or a single dropped/extra byte desyncs forever.
  const uint16_t mark = uart_rx_mark();
  if (!uart_recv_byte(&ser_rx[0], 2U)) { return; }
  if (ser_rx[0] != SERIAL_SYNC_BYTE) { return; }   // not aligned: drop 1 byte, retry next tick

  // read the remaining 6 header bytes. Keep this SHORT: at 1.5 Mbaud a full
  // header takes ~47 us, so 3 ms is generous. A long timeout here parks the
  // whole main loop when the host sends a stray SYNC byte.
#define SERIAL_HDR_TIMEOUT_MS 3U
  if (!uart_recv_raw(&ser_rx[1], SERIAL_HEADER_SIZE - 1U, SERIAL_HDR_TIMEOUT_MS)) {
    // 3 ms is 450 byte-times: a real header is never this late, so that SYNC was
    // payload data. Realign past it instead of flushing what came after.
    serial_stats.hdr_timeout += 1U;
    uart_rx_rewind_past(mark);
    return;
  }
  if (serial_checksum(ser_rx, SERIAL_HEADER_SIZE) != 0U) {
    // Misalignment, not corruption -- the DMA stream is lossless. Step one byte
    // and re-hunt. Deliberately SILENT: a NACK for a header the host never sent
    // lands in its receive stream and gets read as the ack of a LATER
    // transaction, which is how one bad byte became a self-sustaining cascade.
    serial_stats.hdr_resync += 1U;
    (void)memcpy(serial_stats.last_bad_hdr, ser_rx, SERIAL_HEADER_SIZE);
    for (uint16_t i = 0U; i < SERIAL_HEADER_SIZE; i++) {
      serial_stats.reread_hdr[i] = uart_rx_peek(mark, i);
    }
    uart_rx_rewind_past(mark);
    return;
  }
  uint8_t  endpoint  = ser_rx[1];
  uint16_t mosi_len  = (uint16_t)ser_rx[2] | ((uint16_t)ser_rx[3] << 8);
  uint16_t miso_len  = (uint16_t)ser_rx[4] | ((uint16_t)ser_rx[5] << 8);
  if (mosi_len > (SERIAL_BUF_SIZE - SERIAL_HEADER_SIZE - 1U)) {
    // aligned, so the host is genuinely mid-transaction and is waiting on a
    // reply -- NACK it. No flush: the stream is still in step.
    serial_stats.mosi_oversize += 1U;
    uint8_t nack = SERIAL_NACK; uart_send_raw(&nack, 1U); return;
  }
  // Clamp the requested response size to what ser_tx can actually hold.
  // The response is [HACK][len:2][data][checksum], so data may use at most
  // SERIAL_BUF_SIZE - 4. pandad asks for RECV_SIZE (0x4000 = 16384) on the CAN
  // read endpoint, which is 8x this buffer: unclamped, comms_can_read() writes
  // ~14KB past ser_tx and corrupts whatever follows it in RAM. Clamping just
  // returns a shorter batch, which the host already handles -- comms_can_read()
  // keeps the remainder queued for the next call.
  if (miso_len > (SERIAL_BUF_SIZE - SERIAL_RESP_OVERHEAD)) {
    miso_len = SERIAL_BUF_SIZE - SERIAL_RESP_OVERHEAD;
  }

  // 2) ack header, then read mosi data (if any) + its checksum byte
  uint8_t hack = SERIAL_HACK; uart_send_raw(&hack, 1U);
  if (mosi_len > 0U) {
    if (!uart_recv_raw(&ser_rx[SERIAL_HEADER_SIZE], mosi_len + 1U, 25U)) {
      // host abandoned the transaction. Leave the ring alone; whatever arrives
      // next gets realigned by the SYNC hunt above.
      serial_stats.mosi_timeout += 1U;
      return;
    }
    if (serial_checksum(&ser_rx[SERIAL_HEADER_SIZE], mosi_len + 1U) != 0U) {
      // we consumed exactly mosi_len+1 bytes, so the stream is still aligned.
      serial_stats.mosi_checksum += 1U;
      uint8_t nack = SERIAL_NACK; uart_send_raw(&nack, 1U); return;
    }
  }

  // 3) dispatch to comms_* handlers (same as spi.h). resp data goes at ser_tx[3].
  uint16_t resp_len = 0U; bool ack = false;
  if (endpoint == 0U) {
    if (mosi_len >= sizeof(ControlPacket_t)) {
      ControlPacket_t ctrl = {0};
      (void)memcpy((uint8_t*)&ctrl, &ser_rx[SERIAL_HEADER_SIZE], sizeof(ControlPacket_t));
      resp_len = comms_control_handler(&ctrl, &ser_tx[3]); ack = true;
    }
  } else if ((endpoint == 1U) || (endpoint == 0x81U)) {
    if (mosi_len == 0U) { resp_len = comms_can_read(&ser_tx[3], miso_len); ack = true; }
  } else if (endpoint == 2U) {
    comms_endpoint2_write(&ser_rx[SERIAL_HEADER_SIZE], mosi_len); ack = true;
  } else if (endpoint == 3U) {
    if (mosi_len > 0U) { comms_can_write(&ser_rx[SERIAL_HEADER_SIZE], mosi_len); ack = true; }
  } else {
    // unknown endpoint
  }

  // 4) respond: [HACK/NACK][resp_len:u16][resp data][checksum]
  if (ack) {
    ser_tx[0] = SERIAL_HACK;
    ser_tx[1] = resp_len & 0xFFU;
    ser_tx[2] = (resp_len >> 8) & 0xFFU;
    uint16_t total = 3U + resp_len;
    ser_tx[total] = serial_checksum(ser_tx, total);
    uart_send_raw(ser_tx, total + 1U);
    serial_stats.txn_ok += 1U;
  } else {
    uint8_t nack = SERIAL_NACK; uart_send_raw(&nack, 1U);
  }
}

void serial_comms_init(void) {
  // bring up USART2 (PA2/PA3 -> ST-Link VCP), hand the receiver to DMA, then
  // discard whatever the line held while we were coming up. RX must be DMA-fed:
  // see the note in serial_uart_raw.h -- polled RXNE loses bytes to the CAN ISRs.
  usart2_init();
  uart_dma_rx_init();
  uart_flush_rx();
}
