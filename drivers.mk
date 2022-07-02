# List all C files here
_DRIVERSSRC = max-m8c/src/ublox.c

_DRIVERSINC = max-m8c/include

DRIVERSSRC := $(_DRIVERSSRC:%=$(DRIVERSPATH)/%)
DRIVERSINC := $(_DRIVERSINC:%=$(DRIVERSPATH)/%)
