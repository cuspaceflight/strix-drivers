#include "ms5611.h"

#include <stdbool.h>

#include "hal.h"

// &SPID1

static const SPIConfig spi_cfg = {
  false,
  NULL,
  GPIOA,
  4,
  0,
  SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0
};
