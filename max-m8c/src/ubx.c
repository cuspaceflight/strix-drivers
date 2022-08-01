/*
 * UBX Protocol Decoding
 * 2022 CU Spaceflight
 */

#include "ublox.h"


#include <stdint.h>

#include "hal.h"
#include "chprintf.h"

#include "ubx.h"

#include "packets/ack.h"
#include "packets/cfg.h"
#include "packets/nav.h"


static inline void fletcher_step(ubx_checksum_t* checksum, uint8_t next)
{
  checksum->ck_a += next;
  checksum->ck_b += checksum->ck_a;

  return;
}

ubx_checksum_t ubx_calc_checksum(ubx_header_t* header, uint8_t* payload)
{
  ubx_checksum_t ck;
  uint16_t i;

  /* first iterate over the header */
  for (i = 2; i < sizeof(ubx_header_t); i++)
    fletcher_step(&ck, header->data[i]);

  /* then iterate over the payload */
  for (i = 0; i < header->length; i++)
    fletcher_step(&ck, payload[i]);

  /* checksum is modified in place, so finally return it */
  return ck;
}

void ubx_add_checksum(uint8_t* packet)
{
  ubx_header_t* header;
  ubx_checksum_t ck;
  size_t ck_pos;

  header = (ubx_header_t*) packet;

  ck = ubx_calc_checksum(header, (packet + sizeof(ubx_header_t)));

  ck_pos = sizeof(ubx_header_t) + header->length;

  packet[ck_pos]   = ck.ck_a;
  packet[ck_pos+1] = ck.ck_b;

  return;
}
