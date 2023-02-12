/*
 * rxm.h
 *
 *  Created on: 12 Feb 2023
 *      Author: Harry
 */

#ifndef RXM_H_
#define RXM_H_

#define UBX_CLASS_RXM  0x02

#define UBX_RXM_SVSI   0x20

typedef struct __attribute__((packed)) {
  uint32_t i_tow;
  int16_t week;
  uint8_t num_vis, num_sv;
} ubx_rxm_svsi_t;

#endif /* RXM_H_ */
