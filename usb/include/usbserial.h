#ifndef USB_SERIAL_H_
#define USB_SERIAL_H_

#include "ch.h"
#include "hal.h"

#include "shell.h"
#include "chprintf.h"

void usb_init(void);
void usb_thd_init(void);

void usb_test(void);

#endif /* USB_SERIAL_H_ */
