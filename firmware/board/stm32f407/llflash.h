bool flash_is_locked(void) {
  return (FLASH->CR & FLASH_CR_LOCK);
}

void flash_unlock(void) {
  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
}

bool flash_erase_sector(uint8_t sector, bool unlocked) {
  // don't erase the bootloader(sector 0)
  //
  // `sector < 12` is the F4 family bound, not this part's. The 512KB F407VET6
  // has only sectors 0-7: 0-3 are 16KB, 4 is 64KB, 5-7 are 128KB. Sectors
  // 8-11 exist only on 1MB parts (F407VG). Asking for 8-11 here addresses
  // flash that is not there.
  if (sector != 0 && sector < 12 && unlocked) {
    FLASH->CR = (sector << 3) | FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY);
    return true;
  }
  return false;
}

void flash_write_word(void *prog_ptr, uint32_t data) {
  uint32_t *pp = prog_ptr;
  FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;
  *pp = data;
  while (FLASH->SR & FLASH_SR_BSY);
}

void flush_write_buffer(void) { }
