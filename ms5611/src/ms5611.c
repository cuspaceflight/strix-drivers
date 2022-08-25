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
  .cr1      = SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2 | SPI_CR1_CPOL | SPI_CR1_CPHA,
  .cr2      = 0
};

static SPIDriver* ms5611_driver = &SPID1;

static const uint8_t reset_cmd = MS5611_RESET;
static const uint8_t adc_cmd   = MS5611_ADC_READ;

static uint16_t coeffs[6] = {0};

void ms5611_send_cmd(const uint8_t* cmd)
{
  spiSelect(ms5611_driver);
  spiSend(ms5611_driver, 1, cmd);
  spiUnselect(ms5611_driver);
  return;
}

void ms5611_send_cmd_delay(const uint8_t* cmd, int delay)
{
  spiSelect(ms5611_driver);
  spiSend(ms5611_driver, 1, cmd);
  chThdSleepMilliseconds(delay);
  spiUnselect(ms5611_driver);
  return;
}

void ms5611_read_cmd(const uint8_t* cmd, size_t rx_size, uint8_t* rx_buf)
{
  spiSelect(ms5611_driver);
  spiSend(ms5611_driver, 1, cmd);

  if (rx_size == 0 || rx_buf == NULL) {
    spiUnselect(ms5611_driver);
    return;
  }

  spiReceive(ms5611_driver, rx_size, rx_buf);

  spiUnselect(ms5611_driver);
  return;
}

void ms5611_reset(void)
{
  ms5611_send_cmd_delay(&reset_cmd, 5);
}

void ms5611_init(void) {
#if DEBUG
  DEBUG_PRINTLN("Initialising MS5611 (baro) Module...");
#endif

  spiStart(ms5611_driver, &spi_cfg);
  ms5611_reset();

#if DEBUG
  DEBUG_PRINTLN("MS5611 (baro) Module Init Complete!");
  DEBUG_PRINTLN();
#endif
}

float ms5611_convert_d1(uint8_t osr)
{
  bitfield_ms5611_conversion_t cmd = {
    .cov = 1,
    .typ = 0,
    .os  = osr
  };

  ms5611_send_cmd_delay((const uint8_t*) &cmd, 10);
  return .0f;
}

float ms5611_convert_d2(uint8_t osr)
{
  bitfield_ms5611_conversion_t cmd = {
      .cov = 1,
      .typ = 1,
      .os  = osr
    };

  ms5611_send_cmd_delay((const uint8_t*) &cmd, 10);
    return .0f;
}

uint32_t ms5611_adc_read(void)
{
  uint32_t result;
  ms5611_read_cmd(&adc_cmd, 3, (uint8_t*) &result);

  DEBUG_PRINT("[BARO] ADC result: %x\r\n", result);

  return result;
}

uint16_t ms5611_prom_read(uint8_t addr)
{
  union {
    bitfield_ms5611_conversion_t bitfield;
    uint8_t raw;
  } cmd;

  uint16_t result;

  cmd.raw = MS5611_PROM_READ;

//  cmd.bitfield.os = (addr & 0b001) << 2 |
//                    (addr & 0b010)      |
//                    (addr & 0b100) >> 2;

  cmd.bitfield.os = addr & 0b111;

  DEBUG_PRINT("cmd: %x\r\n", cmd.raw);
  ms5611_read_cmd(&(cmd.raw), 2, (uint8_t*) &result);

  return result;
}

void ms5611_init_thd(void)
{
#if DEBUG
  DEBUG_PRINTLN("Initialising MS5611 (baro) Thread...");
#endif

  uint16_t sn = ms5611_prom_read(0b000); /* Serial number? */
#if DEBUG
  DEBUG_PRINT("S/N: %x\r\n", sn);
#endif

  for (uint8_t i = 0; i < 6; i++) {
    coeffs[i] = ms5611_prom_read(i + 1);
#if DEBUG
    DEBUG_PRINT("Coeff %d: %x\r\n", i+1, coeffs[i]);
#endif
  }

  return;

  ms5611_convert_d1(MS5611_OSR_4096);
  ms5611_adc_read();

  ms5611_convert_d1(MS5611_OSR_4096);
  ms5611_adc_read();
  ms5611_adc_read();
  ms5611_adc_read();

  ms5611_convert_d2(MS5611_OSR_4096);
  ms5611_adc_read();
  ms5611_adc_read();

  ms5611_convert_d2(MS5611_OSR_4096);
  ms5611_adc_read();

#if DEBUG
  DEBUG_PRINTLN("MS5611 (baro) Thread Init Complete!");
  DEBUG_PRINTLN();
#endif
}
