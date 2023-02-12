/*
 * ubx.h
 */

#ifndef UBX_H_
#define UBX_H_


#include "packets/ack.h"
#include "packets/cfg.h"
#include "packets/nav.h"
#include "packets/rxm.h"

/* UBX sync bytes */
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

#define NMEA_SYNC 0x24     /* $  */
#define NMEA_DESYNC_1 0x0D /* CR */
#define NMEA_DESYNC_2 0x0A /* LF */


/* UBX packet header */
typedef struct __attribute__((packed)) {
  union {
    uint8_t data[6];
    struct {
      uint8_t sync1,   /* sync char 1 (UBX_SYNC1) */
              sync2,   /* sync char 2 (UBX_SYNC2) */
              class,   /* 1-byte message class    */
              id;      /* 1-byte message ID       */
      uint16_t length; /* 2-byte payload length   */
    } __attribute__((packed));
  };
} ubx_header_t;

/* UBX packet checksum */
typedef struct __attribute__((packed)) {
  union {
    uint8_t data[2];
    struct {
      uint8_t ck_a, ck_b; /* 2-byte UBX checksum */
    } __attribute__((packed));
  };
} ubx_checksum_t;


/* checksum calculation */
ubx_checksum_t ubx_calc_checksum(ubx_header_t* header, uint8_t* payload);

/* add checksum to packet */
void ubx_add_checksum(uint8_t* packet);

#endif // UBX_H_
