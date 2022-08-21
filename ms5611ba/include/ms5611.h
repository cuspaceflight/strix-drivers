#ifndef MS5611_H_
#define MS5611_H_

#define MS5611_RESET 0x1E
#define MS5611_ADC_READ 0x00
#define MS5611_PROM_READ 0x0A


typedef struct __attribute__((packed)) {
    uint8_t : 1;
    uint8_t COV: 1;
    uint8_t : 1;
    uint8_t Typ: 1;
    uint8_t Ad: 3; /* NOTE: msb -> lsb is A2, A1, A0 */
    uint8_t : 1;
} bitfield_ms5611_conversion_t;

#endif