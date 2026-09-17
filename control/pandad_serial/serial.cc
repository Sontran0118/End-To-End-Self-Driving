// PandaSerialHandle: pandad comms over a UART/serial link (Nucleo-F446 panda via ST-Link VCP).
// Implements the same interface as PandaSpiHandle, using the panda serial framing (matches the
// firmware's board/drivers/serial_comms.h):
//   header  = [SYNC=0x5A][endpoint][tx_len:u16 LE][rx_len:u16 LE][hdr_checksum]  (7 bytes)
//   -> device replies 1 byte HACK(0x79)/NACK(0x1F)
//   if tx_len>0: send [tx data][data_checksum]; wait HACK/NACK
//   device replies [HACK][resp_len:u16][resp data][resp_checksum]
#include "selfdrive/pandad/panda_comms.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cassert>
#include <vector>
#include <glob.h>

#define SER_SYNC 0x5A
#define SER_HACK 0x79
#define SER_NACK 0x1F
#define SER_CKSTART 0xAB
#define SER_HDR 7
#define SER_BAUD B1500000

static uint8_t ser_checksum(const uint8_t *d, int n) {
  uint8_t c = SER_CKSTART; for (int i = 0; i < n; i++) c ^= d[i]; return c;
}

PandaSerialHandle::PandaSerialHandle(std::string serial) : hw_serial(serial) {
  std::string dev = serial.empty() ? "/dev/ttyACM0" : serial;
  fd = open(dev.c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0) { connected = false; return; }
  struct termios t; tcgetattr(fd, &t);
  cfmakeraw(&t);
  cfsetispeed(&t, SER_BAUD); cfsetospeed(&t, SER_BAUD);
  t.c_cflag |= CLOCAL | CREAD; t.c_cflag &= ~CRTSCTS;
  t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 1;   // 100ms read timeout
  tcsetattr(fd, TCSANOW, &t);
  tcflush(fd, TCIOFLUSH);
}
PandaSerialHandle::~PandaSerialHandle() { if (fd >= 0) close(fd); }

static bool read_exact(int fd, uint8_t *b, int n) {
  int got = 0; while (got < n) { int r = read(fd, b + got, n - got); if (r <= 0) return false; got += r; }
  return true;
}

// one transaction: send framed request, read framed response. returns resp data length or -1.
int PandaSerialHandle::transfer(uint8_t endpoint, uint8_t *tx, uint16_t tx_len,
                                uint8_t *rx, uint16_t max_rx, unsigned int timeout) {
  std::lock_guard<std::recursive_mutex> lock(hw_lock);
  uint8_t hdr[SER_HDR];
  hdr[0] = SER_SYNC; hdr[1] = endpoint;
  hdr[2] = tx_len & 0xFF; hdr[3] = tx_len >> 8;
  hdr[4] = max_rx & 0xFF; hdr[5] = max_rx >> 8;
  // checksum byte so the device's serial_checksum over all 7 bytes == 0.
  // serial_checksum = SER_CKSTART XOR b[0..5] XOR b[6]; solve b[6] = SER_CKSTART XOR b[0..5].
  hdr[6] = SER_CKSTART; for (int i = 0; i < 6; i++) hdr[6] ^= hdr[i];

  if (write(fd, hdr, SER_HDR) != SER_HDR) { comms_healthy = false; return -1; }
  uint8_t ack;
  if (!read_exact(fd, &ack, 1) || ack != SER_HACK) return -1;

  if (tx_len > 0) {
    write(fd, tx, tx_len);
    // device checks serial_checksum([data..][dck]) == 0 -> dck = SER_CKSTART XOR data
    uint8_t dck = SER_CKSTART; for (int i = 0; i < tx_len; i++) dck ^= tx[i];
    write(fd, &dck, 1);
  }
  // response: [HACK][len lo][len hi][data][cksum]
  uint8_t rhdr[3];
  if (!read_exact(fd, rhdr, 3)) return -1;
  if (rhdr[0] != SER_HACK) return -1;
  uint16_t rlen = rhdr[1] | (rhdr[2] << 8);
  if (rlen > max_rx) rlen = max_rx;
  if (rlen > 0) { if (!read_exact(fd, rx, rlen)) return -1; }
  uint8_t rck; read_exact(fd, &rck, 1);   // trailing checksum (not validated for now)
  return rlen;
}

int PandaSerialHandle::control_write(uint8_t request, uint16_t p1, uint16_t p2, unsigned int timeout) {
  ControlPacket_t pkt = {.request = request, .param1 = p1, .param2 = p2, .length = 0};
  return transfer(0, (uint8_t*)&pkt, sizeof(pkt), nullptr, 0, timeout);
}
int PandaSerialHandle::control_read(uint8_t request, uint16_t p1, uint16_t p2, unsigned char *data, uint16_t length, unsigned int timeout) {
  ControlPacket_t pkt = {.request = request, .param1 = p1, .param2 = p2, .length = length};
  return transfer(0, (uint8_t*)&pkt, sizeof(pkt), data, length, timeout);
}
int PandaSerialHandle::bulk_write(unsigned char endpoint, unsigned char *data, int length, unsigned int timeout) {
  return transfer(endpoint, data, length, nullptr, 0, timeout);
}
int PandaSerialHandle::bulk_read(unsigned char endpoint, unsigned char *data, int length, unsigned int timeout) {
  return transfer(endpoint, nullptr, 0, data, length, timeout);
}
void PandaSerialHandle::cleanup() { if (fd >= 0) { close(fd); fd = -1; } }

std::vector<std::string> PandaSerialHandle::list() {
  std::vector<std::string> ret;
  glob_t g; if (glob("/dev/ttyACM*", 0, nullptr, &g) == 0) {
    for (size_t i = 0; i < g.gl_pathc; i++) ret.push_back(g.gl_pathv[i]);
  }
  globfree(&g);
  return ret;
}
