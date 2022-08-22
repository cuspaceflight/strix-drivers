#include "ms5611.h"

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"

#include "debug.h"

static SPIConfig spi_cfg = {
  .circular = false,
  .data_cb   = NULL,
  .error_cb  = NULL,
  .ssport   = GPIOA,
  .sspad    = 4,
  .cr1      = SPI_CR1_BR_0,
  .cr2      = 0
};

static const uint8_t reset_cmd = MS5611_RESET;
static const uint8_t adc_cmd   = MS5611_ADC_READ;

void ms5611_send_cmd(const uint8_t* cmd)
{
  spiSelect(&SPID1);
  spiSend(&SPID1, 1, cmd);
  spiUnselect(&SPID1);
}

void ms5611_read_cmd(const uint8_t* cmd, size_t rx_size, uint8_t* rx_buf)
{
  spiSelect(&SPID1);
  spiSend(&SPID1, 1, cmd);

  if (rx_size == 0 || rx_buf == NULL) {
    spiUnselect(&SPID1);
    return;
  }

  spiReceive(&SPID1, rx_size, rx_buf);

  spiUnselect(&SPID1);
}

void ms5611_init(void) {
  spiStart(&SPID1, &spi_cfg);
}

void ms5611_reset(void)
{
  spiSelect(&SPID1);
  spiSend(&SPID1, 1, &reset_cmd);
  chThdSleepMilliseconds(300);
  spiUnselect(&SPID1);
}

float ms5611_convert_d1(uint8_t osr)
{
  bitfield_ms5611_conversion_t cmd = {
    .cov = 1,
    .typ = 0,
    .os  = osr
  };

  ms5611_send_cmd((const uint8_t*) &cmd);
  return .0f;
}

float ms5611_convert_d2(uint8_t osr)
{
  bitfield_ms5611_conversion_t cmd = {
      .cov = 1,
      .typ = 1,
      .os  = osr
    };

    ms5611_send_cmd((const uint8_t*) &cmd);
    return .0f;
}

uint32_t ms5611_adc_read(void)
{
  uint8_t rx_buf[3];
  ms5611_read_cmd(&adc_cmd, 3, rx_buf);

  DEBUG_PRINT("%x %x %x", rx_buf[0], rx_buf[1], rx_buf[2]);

  return 0;
}

//uint16_t ms5611_prom_read(void)
//{
//  ms5611_run_cmd(MS5611_PROM_READ);
//}
