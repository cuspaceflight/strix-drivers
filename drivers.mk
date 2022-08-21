# List all C files here
_DRIVERSSRC = max-m8c/src/ublox.c \
			  max-m8c/src/ubx.c \
			  \
			  usb/src/usbserial.c \
			  usb/src/usbcfg.c

_DRIVERSINC = max-m8c/include \
			  usb/include

DRIVERSSRC := $(_DRIVERSSRC:%=$(DRIVERSPATH)/%)
DRIVERSINC := $(_DRIVERSINC:%=$(DRIVERSPATH)/%)
