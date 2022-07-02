# List all C files here
_DRIVERSSRC = 

DRIVERSSRC := $(_DRIVERSSRC:%=$(DRIVERSPATH)/src/%)
DRIVERSINC := $(DRIVERSPATH)/include
