# List all C files here
_DRIVERSSRC = max-m8c/src/ublox.c \
			  max-m8c/src/ubx.c

_DRIVERSINC = max-m8c/include

DRIVERSSRC := $(_DRIVERSSRC:%=$(DRIVERSPATH)/%)
DRIVERSINC := $(_DRIVERSINC:%=$(DRIVERSPATH)/%)
