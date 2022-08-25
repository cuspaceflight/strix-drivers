#ifndef MS5611_H_
#define MS5611_H_

#include <stdint.h>

#define MS5611_RESET 0x1E
#define MS5611_ADC_READ 0x00
#define MS5611_PROM_READ 0xA0

/* NB: these bits are arranged as bits 4, 5, 6 from left to right */
#define MS5611_OSR_256  0b000
#define MS5611_OSR_512  0b100
#define MS5611_OSR_1024 0b010
#define MS5611_OSR_2048 0b110
#define MS5611_OSR_4096 0b001


typedef struct __attribute__((packed)) {
    uint8_t : 1;
    uint8_t os: 3; /* NOTE: msb -> lsb is A2, A1, A0 */
    uint8_t typ: 1;
    uint8_t : 1;
    uint8_t cov: 1;
    uint8_t : 1;
} bitfield_ms5611_conversion_t;

void ms5611_init(void);
void ms5611_init_thd(void);

#endif
