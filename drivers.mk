# List all C files here
_DRIVERSSRC = max-m8c/src/ublox.c   \
			  max-m8c/src/ubx.c     \
			                        \
			  ms5611/src/ms5611.c   \
			                        \
			  usb/src/usbserial.c   \
			  usb/src/usbcfg.c

_DRIVERSINC = max-m8c/include       \
			  ms5611/include        \
			  usb/include

DRIVERSSRC := $(_DRIVERSSRC:%=$(DRIVERSPATH)/%)
DRIVERSINC := $(_DRIVERSINC:%=$(DRIVERSPATH)/%)
