/*
 * nav.h
 */

#ifndef NAV_H_
#define NAV_H_

#define UBX_CLASS_NAV  0x01

#define UBX_NAV_POSECEF 0x01
#define UBX_NAV_PVT     0x07

/* PACKET FORMAT DEFINITIONS */

/* UBX-NAV-POSECEF
 * Position solution in ECEF
 */
typedef struct __attribute__((packed)) {
  uint32_t i_tow;
  int32_t lon, lat, height;
  uint32_t p_acc;
} ubx_nav_posecef_t;


/* UBX-NAV-PVT
 * Contains fix quality, position and time information.
 * Everything you want in one message.
 */
typedef struct __attribute__((packed)) {
  uint32_t i_tow;
  uint16_t year;
  uint8_t month, day, hour, minute, second;
  uint8_t valid;
  uint32_t t_acc;
  int32_t nano;
  uint8_t fix_type;
  uint8_t flags;
  uint8_t reserved1;
  uint8_t num_sv;
  int32_t lon, lat;
  int32_t height, h_msl;
  uint32_t h_acc, v_acc;
  int32_t velN, velE, velD, gspeed;
  int32_t head_mot;
  uint32_t s_acc;
  uint32_t head_acc;
  uint16_t p_dop;
  uint16_t reserved2;
  uint32_t reserved3;
  int32_t head_veh;
  uint32_t reserved4;
} ubx_nav_pvt_t;

#endif // NAV_H_
