/*
 * mpu9250.h
 *
 *  Created on: 13 Feb 2023
 *      Author: Harry
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#define MPU9250_SPID        SPID1
#define MPU9250_SPI_CS_PORT GPIOB
#define MPU9250_SPI_CS_PIN  0

#define MPU9250_REG_WHOAMI          0x75

#define MPU9250_REG_PWR_MGMT_1      0x6B
#define MPU9250_REG_PWR_MGMT_2      0x6C
#define MPU9250_REG_USER_CTRL       0x6A
#define MPU9250_REG_SMPLRT_DIV      0x19
#define MPU9250_REG_CONFIG          0x1A

#define MPU9250_REG_GYRO_CONFIG     0x1B

#define MPU9250_REG_ACCEL_CONFIG    0x1C
#define MPU9250_REG_ACCEL_CONFIG_2  0x1D

#define MPU9250_REG_FIFO_EN         0x23
#define MPU9250_REG_FIFO_COUNTH     0x72
#define MPU9250_REG_FIFO_COUNTL     0x73
#define MPU9250_REG_FIFO_R_W        0x74

typedef struct __attribute__((packed)) { uint16_t x, y, z; } mpu9250_accel_t;
typedef struct __attribute__((packed)) { uint16_t x, y, z; } mpu9250_gyro_t;

typedef struct __attribute__((packed)) {
  mpu9250_accel_t accel;
  mpu9250_gyro_t  gyro;
} mpu9250_fifo_t;

void mpu9250_init(void);

uint8_t mpu9250_whoami(void);

uint16_t mpu9250_fifo_count(void);
mpu9250_fifo_t mpu9250_pop_fifo(void);

#endif /* MPU9250_H_ */
