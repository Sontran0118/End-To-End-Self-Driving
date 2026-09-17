/*
  CAN transactions to and from the host come in the form of
  a certain number of CANPacket_t. The transaction is split
  into multiple transfers or chunks.

  * comms_can_read outputs this buffer in chunks of a specified length.
    chunks are always the given length, except the last one.
  * comms_can_write reads in this buffer in chunks.
  * both functions maintain an overflow buffer for a partial CANPacket_t that
    spans multiple transfers/chunks.
  * the overflow buffers are reset by a dedicated control transfer handler,
    which is sent by the host on each start of a connection.
*/

// Per-packet sync marker for the bulk IN stream. See the long note in
// comms_can_read.
//
// 0xAA IS NOT ARBITRARY. The first byte of a packet encodes bus in bits 3:1, and
// a packet is only valid when bus <= 2. 0xAA gives bus = 5, so it can NEVER be
// the first byte of a legal packet -- which means a host parsing an UNMARKED
// stream loses nothing by treating 0xAA as a marker candidate, and the two
// formats can coexist without ambiguity. Only 96 of 256 byte values can legally
// start a packet; any of the other 160 would have done.
//
// The marker can still occur inside a PAYLOAD, so it is not proof on its own.
// The host requires marker + valid header + valid XOR and chains that across
// consecutive packets. The marker's job is to make the re-lock DETERMINISTIC
// rather than a guess.
#define CAN_SYNC_MARKER 0xAAU

typedef struct {
  uint32_t ptr;
  uint32_t tail_size;
  uint8_t data[72];
} asm_buffer;

static asm_buffer can_read_buffer = {.ptr = 0U, .tail_size = 0U};

int comms_can_read(uint8_t *data, uint32_t max_len) {
  uint32_t pos = 0U;

  // Send tail of previous message if it is in buffer
  if (can_read_buffer.ptr > 0U) {
    uint32_t overflow_len = MIN(max_len - pos, can_read_buffer.ptr);
    (void)memcpy(&data[pos], can_read_buffer.data, overflow_len);
    pos += overflow_len;
    (void)memcpy(can_read_buffer.data, &can_read_buffer.data[overflow_len], can_read_buffer.ptr - overflow_len);
    can_read_buffer.ptr -= overflow_len;
  }

  if (can_read_buffer.ptr == 0U) {
    // Fill rest of buffer with new data
    CANPacket_t can_packet;
    while ((pos < max_len) && can_pop(&can_rx_q, &can_packet)) {
      // SYNC MARKER (Jetson port). Each packet is prefixed with CAN_SYNC_MARKER
      // so the host can re-lock onto a frame boundary DETERMINISTICALLY.
      //
      // WHY. This stream is packets concatenated with no delimiter. When a byte
      // is lost the host must guess the alignment, and its only tests are
      // bus<=2, an address range and an 8-bit XOR. Those are weak enough that a
      // WRONG alignment passes often, and on a repetitive bus the parser then
      // locks onto it and keeps finding "valid" packets forever -- advancing by
      // the wrong stride, decoding payload bytes as headers.
      //
      // MEASURED ON THE CAR 2026-08-13: it ran in that state for FOUR MINUTES
      // at full throughput (can rx 2316/s before, 2320/s after) with every
      // decoded value garbage. Radar suppression and lateral both stopped, and
      // no watchdog could see it because nothing was slow or missing.
      //
      // With a marker the host requires MARKER + valid header + valid XOR, and
      // chains that check packet to packet, so a false lock has to survive an
      // improbable coincidence at exactly the right stride, repeatedly.
      //
      // Cost is one byte per packet: ~2.9 KB/s at this bus's 2900 frames/s,
      // negligible on the USB link.
      //
      // The host accepts BOTH formats (see _recv_resync in usb_panda.py), so
      // firmware and host can be updated in either order without a flag day.
      uint8_t framed[1U + sizeof(CANPacket_t)];
      uint32_t pckt_len = 1U + CANPACKET_HEAD_SIZE + dlc_to_len[can_packet.data_len_code];
      framed[0] = CAN_SYNC_MARKER;
      (void)memcpy(&framed[1], (uint8_t*)&can_packet, pckt_len - 1U);
      if ((pos + pckt_len) <= max_len) {
        (void)memcpy(&data[pos], framed, pckt_len);
        pos += pckt_len;
      } else {
        (void)memcpy(&data[pos], framed, max_len - pos);
        can_read_buffer.ptr += pckt_len - (max_len - pos);
        // cppcheck-suppress objectIndex
        (void)memcpy(can_read_buffer.data, &framed[(max_len - pos)], can_read_buffer.ptr);
        pos = max_len;
      }
    }
  }

  return pos;
}

static asm_buffer can_write_buffer = {.ptr = 0U, .tail_size = 0U};

// send on CAN
void comms_can_write(const uint8_t *data, uint32_t len) {
  uint32_t pos = 0U;

  // Assembling can message with data from buffer
  if (can_write_buffer.ptr != 0U) {
    if (can_write_buffer.tail_size <= (len - pos)) {
      // we have enough data to complete the buffer
      CANPacket_t to_push = {0};
      (void)memcpy(&can_write_buffer.data[can_write_buffer.ptr], &data[pos], can_write_buffer.tail_size);
      can_write_buffer.ptr += can_write_buffer.tail_size;
      pos += can_write_buffer.tail_size;

      // send out
      (void)memcpy((uint8_t*)&to_push, can_write_buffer.data, can_write_buffer.ptr);
      can_send(&to_push, to_push.bus, false);

      // reset overflow buffer
      can_write_buffer.ptr = 0U;
      can_write_buffer.tail_size = 0U;
    } else {
      // maybe next time
      uint32_t data_size = len - pos;
      (void) memcpy(&can_write_buffer.data[can_write_buffer.ptr], &data[pos], data_size);
      can_write_buffer.tail_size -= data_size;
      can_write_buffer.ptr += data_size;
      pos += data_size;
    }
  }

  // rest of the message
  while (pos < len) {
    uint32_t pckt_len = CANPACKET_HEAD_SIZE + dlc_to_len[(data[pos] >> 4U)];
    if ((pos + pckt_len) <= len) {
      CANPacket_t to_push = {0};
      (void)memcpy((uint8_t*)&to_push, &data[pos], pckt_len);
      can_send(&to_push, to_push.bus, false);
      pos += pckt_len;
    } else {
      (void)memcpy(can_write_buffer.data, &data[pos], len - pos);
      can_write_buffer.ptr = len - pos;
      can_write_buffer.tail_size = pckt_len - can_write_buffer.ptr;
      pos += can_write_buffer.ptr;
    }
  }

  refresh_can_tx_slots_available();
}

void comms_can_reset(void) {
  can_write_buffer.ptr = 0U;
  can_write_buffer.tail_size = 0U;
  can_read_buffer.ptr = 0U;
  can_read_buffer.tail_size = 0U;
}

// TODO: make this more general!
void refresh_can_tx_slots_available(void) {
  if (can_tx_check_min_slots_free(MAX_CAN_MSGS_PER_USB_BULK_TRANSFER)) {
    can_tx_comms_resume_usb();
  }
  if (can_tx_check_min_slots_free(MAX_CAN_MSGS_PER_SPI_BULK_TRANSFER)) {
    can_tx_comms_resume_spi();
  }
}
