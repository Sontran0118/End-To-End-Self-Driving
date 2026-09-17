#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>


#define TIMEOUT 0
#define SPI_BUF_SIZE 2048


// Common interface implemented by PandaSpiHandle (comma panda) and
// PandaSerialHandle (DIY STM32F446 panda over ST-Link VCP serial).
class PandaCommsHandle {
public:
  std::string hw_serial;
  std::atomic<bool> connected = true;
  std::atomic<bool> comms_healthy = true;

  virtual ~PandaCommsHandle() {}
  virtual int control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout=TIMEOUT) = 0;
  virtual int control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout=TIMEOUT) = 0;
  virtual int bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT) = 0;
  virtual int bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT) = 0;
  virtual void cleanup() = 0;
};

class PandaSpiHandle : public PandaCommsHandle {
public:

  PandaSpiHandle(std::string serial);
  ~PandaSpiHandle();

  int control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout=TIMEOUT);
  int control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout=TIMEOUT);
  int bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);
  int bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT);
  void cleanup();

  static std::vector<std::string> list();

private:
  int spi_fd = -1;
  uint8_t tx_buf[SPI_BUF_SIZE];
  uint8_t rx_buf[SPI_BUF_SIZE];
  inline static std::recursive_mutex hw_lock;

  struct __attribute__((packed)) spi_header {
    uint8_t sync;
    uint8_t endpoint;
    uint16_t tx_len;
    uint16_t max_rx_len;
  };

  int wait_for_ack(uint8_t ack, uint8_t tx, unsigned int timeout, unsigned int length);
  int bulk_transfer(uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t rx_len, unsigned int timeout);
  int spi_transfer(uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t max_rx_len, unsigned int timeout);
  int spi_transfer_retry(uint8_t endpoint, uint8_t *tx_data, uint16_t tx_len, uint8_t *rx_data, uint16_t max_rx_len, unsigned int timeout);
  int lltransfer(struct spi_ioc_transfer &t);

  spi_header header;
  uint32_t xfer_count = 0;
};


// PandaSerialHandle - DIY STM32F446 panda over UART (ST-Link VCP).
// Framing matches the firmware's board/drivers/serial_comms.h.
class PandaSerialHandle : public PandaCommsHandle {
public:
  PandaSerialHandle(std::string serial);
  ~PandaSerialHandle();

  int control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout=TIMEOUT) override;
  int control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout=TIMEOUT) override;
  int bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT) override;
  int bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout=TIMEOUT) override;
  void cleanup() override;

  static std::vector<std::string> list();

private:
  int fd = -1;
  inline static std::recursive_mutex hw_lock;
  // transfer() retries with a flush+resync; transfer_once() is a single attempt.
  int transfer(uint8_t endpoint, uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t max_rx, unsigned int timeout);
  int transfer_once(uint8_t endpoint, uint8_t *tx, uint16_t tx_len, uint8_t *rx, uint16_t max_rx, unsigned int timeout);
};
