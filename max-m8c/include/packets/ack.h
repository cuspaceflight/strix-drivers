/*
 * ack.h
 */

#ifndef ACK_H_
#define ACK_H_

#define UBX_CLASS_ACK  0x05

#define UBX_ACK_NAK     0x00
#define UBX_ACK_ACK     0x01

/* UBX-ACK
 * ACK/NAK messages after trying to set a config.
 */
typedef struct __attribute__((packed)) {
  uint8_t cls_id;
  uint8_t msg_id;
} ubx_ack_t;

#endif // ACK_H_
