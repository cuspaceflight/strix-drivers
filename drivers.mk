# List all C files here
_DRIVERSSRC = max-m8c/src/ublox.c   \
			  max-m8c/src/ubx.c     \
			  						\
			  mpu9250/src/mpu9250.c \
			                        \
			  ms5611/src/ms5611.c   \
			                        \
			  usb/src/usbserial.c   \
			  usb/src/usbcfg.c

_DRIVERSINC = max-m8c/include       \
			  mpu9250/include       \
			  ms5611/include        \
			  usb/include

DRIVERSSRC := $(_DRIVERSSRC:%=$(DRIVERSPATH)/%)
DRIVERSINC := $(_DRIVERSINC:%=$(DRIVERSPATH)/%)
