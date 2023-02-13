#include <stdlib.h>
#include <stdint.h>

#include "mpu9250.h"

#include "hal.h"

#include "debug.h"

static const SPIConfig mpu9250_spi_cfg = {
  .circular = false,
  .data_cb  = NULL,
  .error_cb = NULL,
  .ssport   = GPIOB,
  .sspad    = 0,
  .cr1      = SPI_CR1_BR_2 | SPI_CR1_CPOL | SPI_CR1_CPHA,
  .cr2      = 0
};

static const SPIDriver* mpu9250_driver = &SPID1;

static inline int mpu9250_write(uint8_t addr, uint8_t data)
{
  uint8_t buf[] = { addr, data };

  spiAcquireBus(mpu9250_driver);

  spiSelect(mpu9250_driver);
  spiSend(mpu9250_driver, 2, (void*)&buf);
  spiUnselect(mpu9250_driver);

  spiReleaseBus(mpu9250_driver);
}
static inline int mpu9250_read(uint8_t addr, uint8_t* rx_buf, size_t n)
{
  uint8_t addr_masked = addr | 0x80;
  spiAcquireBus(mpu9250_driver);

  spiSelect(mpu9250_driver);
  spiSend(mpu9250_driver, 1, (void*)&addr_masked);
  spiReceive(mpu9250_driver, n, (void*)rx_buf);
  spiUnselect(mpu9250_driver);

  spiReleaseBus(mpu9250_driver);
}

void mpu9250_init(void)
{
  spiStart(mpu9250_driver, &mpu9250_spi_cfg);

  // Register/value pairs to reset/initialise MPU9250
  static const uint8_t init_sequence[][2] = {
      { MPU9250_REG_PWR_MGMT_1, 0x80 }, // Reset

      { MPU9250_REG_PWR_MGMT_1, 0x01       }, // Select best clock source
      { MPU9250_REG_PWR_MGMT_2, 0x00       }, // Enable gyro & accel
      { MPU9250_REG_USER_CTRL,  0b01010100 },
      { MPU9250_REG_SMPLRT_DIV, 0x0        }, // Set Maximum Sample Rate



      // Set full gyro scale to +500dps
      // We enable a digital low pass filter with:
      // - bandwidth 184Hz on gyroscope
      // - bandwidth 188Hz on temperature
      // Note: this reduces ODR to 1 kHz and adds a 2.9ms delay
      { MPU9250_REG_CONFIG,      0b00000001 }, // Disable FSync and set DLPF_CFG to 1
      { MPU9250_REG_GYRO_CONFIG, 0b00001000 },

      // Set full accelerometer scale to +16g
      // Enable a 184Hz low pass filter on accelerometer
      // Note: this reduces ODR to 1kHz and adds a 5.8ms delay
      { MPU9250_REG_ACCEL_CONFIG,   0b00011000 },
      { MPU9250_REG_ACCEL_CONFIG_2, 0b00000001 },

      { MPU9250_REG_FIFO_EN, 0b01111000 }
  };

  // Perform initial reset
  for(size_t i = 0; i < sizeof(init_sequence) / 2; ++i) {
      mpu9250_write(init_sequence[i][0], init_sequence[i][1]);
      chThdSleepMilliseconds(10);
  }
}

uint8_t mpu9250_whoami(void)
{
  uint8_t whoami;

  mpu9250_read(MPU9250_REG_WHOAMI, &whoami, 1);

  return whoami;
}

uint16_t mpu9250_fifo_count(void)
{
  union {
    struct {
      uint8_t l, h;
    } __attribute__((packed));
    uint16_t x;
  } count;

  mpu9250_read(MPU9250_REG_FIFO_COUNTH, &count.h, 1);
  mpu9250_read(MPU9250_REG_FIFO_COUNTL, &count.l, 1);

  return count.x;
}

mpu9250_fifo_t mpu9250_pop_fifo(void)
{

}
