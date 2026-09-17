#include "llusb_declarations.h"

USB_OTG_GlobalTypeDef *USBx = USB_OTG_FS;

static void OTG_FS_IRQ_Handler(void) {
  NVIC_DisableIRQ(OTG_FS_IRQn);
  //__disable_irq();
  usb_irqhandler();
  //__enable_irq();
  NVIC_EnableIRQ(OTG_FS_IRQn);
}

void usb_init(void) {
  REGISTER_INTERRUPT(OTG_FS_IRQn, OTG_FS_IRQ_Handler, 1500000U, FAULT_INTERRUPT_RATE_USB) //TODO: Find out a better rate limit for USB. Now it's the 1.5MB/s rate

  // full speed PHY, do reset and remove power down
  /*puth(USBx->GRSTCTL);
  print(" resetting PHY\n");*/
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);
  //print("AHB idle\n");

  // reset PHY here
  USBx->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST);
  //print("reset done\n");

  // internal PHY, force device mode
  USBx->GUSBCFG = USB_OTG_GUSBCFG_PHYSEL | USB_OTG_GUSBCFG_FDMOD;

  // slowest timings
  USBx->GUSBCFG |= ((USBD_FS_TRDT_VALUE << 10) & USB_OTG_GUSBCFG_TRDT);

  /* Power up the PHY, and tell the core to stop looking for VBUS.

     This board does NOT wire USB VBUS to the MCU: the FK407M1 routes PA9 to the
     4x2 serial header (as USART1 TX) rather than to the USB connector, so the
     VBUS sense input floats. With VBUS sensing left enabled the core never sees
     a valid B-session, never pulls up D+, and the host sees nothing at all --
     no enumeration, not even a failed attach in dmesg. That failure looks
     exactly like firmware hanging on boot, which is how it was misdiagnosed.

     The F446 handles this by forcing B-session-valid via GOTGCTL BVALOVAL/
     BVALOEN, but the F407's OTG_FS core predates those bits and stm32f407xx.h
     does not define them -- so the #if that used to guard them silently
     compiled the whole thing out, leaving nothing in its place. The equivalent
     on this core revision is GCCFG_NOVBUSSENS, which disables the sensing
     comparators outright.

     ST's own ROM bootloader does the same thing, which is why DFU enumerates
     on this board while our firmware did not. */
#if defined(USB_OTG_GCCFG_NOVBUSSENS)
  USBx->GCCFG = USB_OTG_GCCFG_PWRDWN | USB_OTG_GCCFG_NOVBUSSENS;
#elif defined(USB_OTG_GCCFG_VBDEN)
  // Newer cores (F446/F7): VBUS detect is opt-in, so simply leaving VBDEN clear
  // is the equivalent, but the session still has to be forced valid below.
  USBx->GCCFG = USB_OTG_GCCFG_PWRDWN;
#else
  USBx->GCCFG = USB_OTG_GCCFG_PWRDWN;
#endif

  /* B-peripheral session valid override -- only on cores that have it (F446+).
     Not needed on the F407: NOVBUSSENS above already forces the session valid. */
#if defined(USB_OTG_GOTGCTL_BVALOVAL)
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
#endif

  // be a device, slowest timings
  //USBx->GUSBCFG = USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL | USB_OTG_GUSBCFG_TRDT | USB_OTG_GUSBCFG_TOCAL;
  //USBx->GUSBCFG |= (uint32_t)((USBD_FS_TRDT_VALUE << 10) & USB_OTG_GUSBCFG_TRDT);
  //USBx->GUSBCFG = USB_OTG_GUSBCFG_PHYSEL | USB_OTG_GUSBCFG_TRDT | USB_OTG_GUSBCFG_TOCAL;

  // **** for debugging, doesn't seem to work ****
  //USBx->GUSBCFG |= USB_OTG_GUSBCFG_CTXPKT;

  // reset PHY clock
  USBx_PCGCCTL = 0;

  // enable the fancy OTG things
  // DCFG_FRAME_INTERVAL_80 is 0
  //USBx->GUSBCFG |= USB_OTG_GUSBCFG_HNPCAP | USB_OTG_GUSBCFG_SRPCAP;
  USBx_DEVICE->DCFG |= USB_OTG_SPEED_FULL | USB_OTG_DCFG_NZLSOHSK;

  //USBx_DEVICE->DCFG = USB_OTG_DCFG_NZLSOHSK | USB_OTG_DCFG_DSPD;
  //USBx_DEVICE->DCFG = USB_OTG_DCFG_DSPD;

  // clear pending interrupts
  USBx->GINTSTS = 0xBFFFFFFFU;

  // setup USB interrupts
  // all interrupts except TXFIFO EMPTY
  //USBx->GINTMSK = 0xFFFFFFFF & ~(USB_OTG_GINTMSK_NPTXFEM | USB_OTG_GINTMSK_PTXFEM | USB_OTG_GINTSTS_SOF | USB_OTG_GINTSTS_EOPF);
  //USBx->GINTMSK = 0xFFFFFFFF & ~(USB_OTG_GINTMSK_NPTXFEM | USB_OTG_GINTMSK_PTXFEM);
  USBx->GINTMSK = USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_OTGINT |
                  USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTMSK_GONAKEFFM | USB_OTG_GINTMSK_GINAKEFFM |
                  USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_USBSUSPM |
                  USB_OTG_GINTMSK_CIDSCHGM | USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_MMISM | USB_OTG_GINTMSK_EOPFM;

  USBx->GAHBCFG = USB_OTG_GAHBCFG_GINT;

  // DCTL startup value is 2 on new chip, 0 on old chip
  USBx_DEVICE->DCTL = 0;

  // enable the IRQ
  NVIC_EnableIRQ(OTG_FS_IRQn);
}
