// Raw UART send/recv on USART2 for the panda-over-serial transport.
// USART2 on the Nucleo-F446RE is wired to the ST-Link Virtual COM Port (PA2=TX, PA3=RX).
// APB1 = 45 MHz. Baud is set in peripherals.h init; here we just do byte I/O + timeouts.
//
// TX is polled: losing the CPU mid-send only delays a byte, it cannot corrupt one.
//
// RX is DMA. It used to poll RXNE and read DR, and that does not survive this
// board's CAN load. At 1.5 Mbaud a byte lands every 6.67 us, USART2 has a
// one-byte receive register and no FIFO, and the receive loop runs in the main
// loop -- so any interrupt longer than one byte-time drops a byte to ORE. With
// the acceptance filters passing the whole bus, can_rx() runs ~2300 times/s on
// bus 0 alone (plus the forwarding tx it queues), and measured control-transfer
// reliability fell from 100% to ~62%: every other transaction lost a byte and
// desynced the framing. The old ORE-recovery path below could unwedge the USART
// but could not un-lose the byte.
//
// DMA1 Stream 5 Channel 4 is USART2_RX on the F446. In circular mode it writes
// straight into uart_rx_buf with zero CPU involvement, so an ISR of any length
// costs latency instead of data. NDTR counts down, so the producer index is
// (size - NDTR); we keep our own consumer index.

#ifndef SERIAL_UART
#define SERIAL_UART USART2
#endif

// Power of two: the wrap is a mask, and it must stay cheap -- uart_rx_wr() is
// read in the inner receive loop. 27 ms of buffering at 1.5 Mbaud, against a
// host that sends at most a few hundred bytes per transaction.
#define UART_RX_BUF_SIZE 4096U
// volatile: the DMA writes this behind the compiler's back
static volatile uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
static uint16_t uart_rx_rd;

// DMA write cursor: how far the hardware has filled the ring.
static uint16_t uart_rx_wr(void) {
  return (uint16_t)((UART_RX_BUF_SIZE - (DMA1_Stream5->NDTR & 0xFFFFU)) & (UART_RX_BUF_SIZE - 1U));
}

// Bytes the DMA has captured that we have not consumed yet.
static uint16_t uart_rx_avail(void) {
  return (uint16_t)((uart_rx_wr() - uart_rx_rd) & (UART_RX_BUF_SIZE - 1U));
}

void uart_dma_rx_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

  // stop the stream and wait for it to actually stop before reprogramming
  DMA1_Stream5->CR &= ~DMA_SxCR_EN;
  while ((DMA1_Stream5->CR & DMA_SxCR_EN) != 0U) {}
  DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |
                DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;

  DMA1_Stream5->PAR = (uint32_t)(&(SERIAL_UART->DR));
  DMA1_Stream5->M0AR = (uint32_t)uart_rx_buf;
  DMA1_Stream5->NDTR = UART_RX_BUF_SIZE;
  DMA1_Stream5->FCR = 0U;                              // direct mode, no FIFO
  // channel 4, peripheral->memory, byte/byte, memory increment, circular
  DMA1_Stream5->CR = (4UL << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_CIRC;
  DMA1_Stream5->CR |= DMA_SxCR_EN;

  // clear any latched error state, then hand the receiver to the DMA
  (void)SERIAL_UART->SR;
  (void)SERIAL_UART->DR;
  SERIAL_UART->CR3 |= USART_CR3_DMAR;

  uart_rx_rd = uart_rx_wr();
}

// millisecond deadline helper using the panda's microsecond_timer (already used elsewhere)
static bool uart_wait_flag(volatile uint32_t *sr, uint32_t flag, uint32_t timeout_ms) {
  uint32_t start = microsecond_timer_get();
  while ((*sr & flag) == 0U) {
    if ((microsecond_timer_get() - start) > (timeout_ms * 1000U)) { return false; }
  }
  return true;
}

void uart_send_raw(const uint8_t *d, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    if (!uart_wait_flag(&SERIAL_UART->SR, USART_SR_TXE, 10U)) { return; }
    SERIAL_UART->DR = d[i];
  }
  // wait for transmission complete so the line is idle before we listen again
  (void)uart_wait_flag(&SERIAL_UART->SR, USART_SR_TC, 10U);
}

// Framing counters live here, not in serial_comms.h: this layer bumps rx_late,
// and it is included first. Declared in serial_comms_declarations.h.
serial_stats_t serial_stats;

// Cycles to wait before trusting a freshly-announced ring byte. At 180 MHz this
// is ~0.36 us -- two orders of magnitude more than a bus-matrix write needs, and
// still a twentieth of the 6.67 us between bytes at 1.5 Mbaud.
#define UART_RX_COMMIT_SPIN 64U

// Read a single byte with timeout. Returns false on timeout (nothing consumed).
static bool uart_recv_byte(uint8_t *b, uint32_t timeout_ms) {
  uint32_t start = microsecond_timer_get();
  while (uart_rx_avail() == 0U) {
    if ((microsecond_timer_get() - start) > (timeout_ms * 1000U)) { return false; }
  }

  // NDTR is decremented when the DMA ISSUES the memory write, not when that
  // write has reached SRAM, so a byte the cursor already claims can still be in
  // flight and read back as the ring's PREVIOUS contents. Measured, not guessed:
  // a header byte read as 0x00 and re-read as 0x07 from the same ring address a
  // moment later, the frame's own checksum confirming 0x07 was the true value.
  // It corrupted ~40% of headers -- one byte, mid-frame, no timeout, no overrun.
  //
  // An earlier attempt gated this on the cursor having moved past the byte, on
  // the theory that only the NEWEST byte is at risk. That was wrong: the capture
  // showed byte 2 of a header corrupt while bytes 3-6, read after it, were fine.
  // So don't reason about which byte is exposed -- read, wait a fixed span, read
  // again, and take the second value. rx_late counts the disagreements, so if
  // this is ever NOT the mechanism, the counter says so instead of hiding it.
  const uint8_t first = uart_rx_buf[uart_rx_rd];
  for (uint32_t i = 0U; i < UART_RX_COMMIT_SPIN; i++) { __asm__ volatile("nop"); }
  const uint8_t settled = uart_rx_buf[uart_rx_rd];
  if (first != settled) { serial_stats.rx_late += 1U; }

  *b = settled;
  uart_rx_rd = (uint16_t)((uart_rx_rd + 1U) & (UART_RX_BUF_SIZE - 1U));
  return true;
}

bool uart_recv_raw(uint8_t *d, uint16_t len, uint32_t timeout_ms) {
  for (uint16_t i = 0; i < len; i++) {
    if (!uart_recv_byte(&d[i], timeout_ms)) { return false; }
  }
  return true;
}

// Framing resync primitives. The DMA stream is lossless, so a header that fails
// its checksum means we are MISALIGNED -- not that bytes were corrupted. Mark
// where a candidate header started, and on rejection rewind to one byte past it
// and re-hunt. Do NOT flush: a good header may already be sitting behind the bad
// one, and throwing it away is what turns one desync into a sustained cascade.
static uint16_t uart_rx_mark(void) { return uart_rx_rd; }

// Re-read a byte the consumer already passed, straight out of the ring.
static uint8_t uart_rx_peek(uint16_t mark, uint16_t off) {
  return uart_rx_buf[(mark + off) & (UART_RX_BUF_SIZE - 1U)];
}
static void uart_rx_rewind_past(uint16_t mark) {
  uart_rx_rd = (uint16_t)((mark + 1U) & (UART_RX_BUF_SIZE - 1U));
}

// Drop everything captured so far. Only correct at init, or when the ring has
// genuinely lapped us -- in both cases the buffered bytes are already garbage.
static void uart_flush_rx(void) {
  uart_rx_rd = uart_rx_wr();
}
