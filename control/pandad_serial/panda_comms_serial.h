// PandaSerialHandle declaration — add to selfdrive/pandad/panda_comms.h alongside PandaSpiHandle.
// pandad's connect() should try this handle for /dev/ttyACM* devices (the Nucleo VCP).
#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

class PandaSerialHandle {
public:
  std::string hw_serial;
  std::atomic<bool> connected = true;
  std::atomic<bool> comms_healthy = true;

  PandaSerialHandle(std::string serial);
  ~PandaSerialHandle();

  int control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout=0);
  int control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout=0);
  int bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=0);
  int bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=0);
  void cleanup();

  static std::vector<std::string> list();

private:
  int fd = -1;
  inline static std::recursive_mutex hw_lock;
  int transfer(uint8_t endpoint, uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t max_rx, unsigned int timeout);
};
