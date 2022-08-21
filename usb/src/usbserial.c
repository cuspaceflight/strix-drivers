#include "usbserial.h"

#include "ch.h"
#include "hal.h"

#include "shell.h"
#include "chprintf.h"

#include "usbcfg.h"

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

thread_t* shelltp = NULL;
event_listener_t shell_el;

/* Can be measured using dd if=/dev/xxxx of=/dev/null bs=512 count=10000.*/
static void cmd_write(BaseSequentialStream *chp, int argc, char *argv[]) {
  static uint8_t buf[] =
      "B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5"
      "B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5"
      "B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5"
      "B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5"
      "B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5"
      "B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5"
      "B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5"
      "B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5"
      "B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5"
      "B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5"
      "B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5"
      "B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5"
      "B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5"
      "B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5"
      "B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5"
      "B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5B16B00B5";

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: write\r\n");
    return;
  }

  while (chnGetTimeout((BaseChannel *)chp, TIME_IMMEDIATE) == Q_TIMEOUT) {
    /* Writing in channel mode.*/
    chnWrite(&usb_driver, buf, sizeof buf - 1);
  }
  chprintf(chp, "\r\n\nstopped\r\n");
}

static const ShellCommand commands[] = {
  {"write", cmd_write},
  {NULL, NULL}
};

static const ShellConfig shell_cfg = {
  (BaseSequentialStream *)&usb_driver,
  commands
};

void usb_init(void) {
  sduObjectInit(&usb_driver);
  sduStart(&usb_driver, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  /*
   * Shell manager initialization.
   * Event zero is shell exit.
   */
//  shellInit();
//  chEvtRegister(&shell_terminated, &shell_el, 0);
}

static THD_WORKING_AREA(usb_thd_wa, 128);
static THD_FUNCTION(usb_thd, arg) {

  (void)arg;
  chRegSetThreadName("USB");

  while (true) {
     if (usb_driver.config->usbp->state == USB_ACTIVE) {
       /* Starting shells.*/
       if (shelltp == NULL) {
         shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                        "USB shell", NORMALPRIO + 1,
                                        shellThread, (void *)&shell_cfg);
       }

       /* Waiting for an exit event then freeing terminated shells.*/
       chEvtWaitAny(EVENT_MASK(0));
       if (chThdTerminatedX(shelltp)) {
         chThdRelease(shelltp);
         shelltp = NULL;
       }
     }
     else {
       chThdSleepMilliseconds(1000);
     }
   }
}

/* Init USB Thread */
void usb_thd_init(void)
{
  chThdCreateStatic(usb_thd_wa, sizeof(usb_thd_wa), NORMALPRIO, usb_thd, NULL);
  return;
}

void usb_test(void)
{
  chThdSleepMilliseconds(1000);
  return;
}

