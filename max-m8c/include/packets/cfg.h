/*
 * cfg.h
 */

#ifndef CFG_H_
#define CFG_H_

#define UBX_CLASS_CFG  0x06

#define UBX_CFG_PRT     0x00
#define UBX_CFG_MSG     0x01
#define UBX_CFG_RATE    0x08
#define UBX_CFG_SBAS    0x16
#define UBX_CFG_NAV5    0x24
#define UBX_CFG_GNSS    0x3E


/* GPS SYSTEM IDS */

#define UBX_GNSS_ID_GPS      0x00
#define UBX_GNSS_ID_SBAS     0x01
#define UBX_GNSS_ID_GALILEO  0x02
#define UBX_GNSS_ID_BEIDOU   0x03
#define UBX_GNSS_ID_IMES     0x04
#define UBX_GNSS_ID_QZSS     0x05
#define UBX_GNSS_ID_GLONASS  0x06


/* BITFIELD MEMBER DEFINITIONS */

#define UBX_CONFIG_MODE_CHARLEN_5BIT 0b00
#define UBX_CONFIG_MODE_CHARLEN_6BIT 0b01
#define UBX_CONFIG_MODE_CHARLEN_7BIT 0b10
#define UBX_CONFIG_MODE_CHARLEN_8BIT 0b11

#define UBX_CFG_PRT_MODE_PARITY_EVEN_PARITY 0b000
#define UBX_CFG_PRT_MODE_PARITY_ODD_PARITY  0b001
#define UBX_CFG_PRT_MODE_PARITY_NO_PARITY   0b100
#define UBX_CFG_PRT_MODE_PARITY_RSVD        0b010

#define UBX_CFG_PRT_MODE_NSTOPBITS_1   0b00
#define UBX_CFG_PRT_MODE_NSTOPBITS_1_5 0b01
#define UBX_CFG_PRT_MODE_NSTOPBITS_2   0b10
#define UBX_CFG_PRT_MODE_NSTOPBITS_0_5 0b11


#define UBX_CFG_GNSS_FLAGS_GPS_L1C_A 0x01
#define UBX_CFG_GNSS_FLAGS_GPS_L2C   0x10
#define UBX_CFG_GNSS_FLAGS_GPS_L5    0x20

#define UBX_CFG_GNSS_FLAGS_SBAS_L1C_A 0x01

#define UBX_CFG_GNSS_FLAGS_GALILEO_E1  0x01
#define UBX_CFG_GNSS_FLAGS_GALILEO_E5A 0x10
#define UBX_CFG_GNSS_FLAGS_GALILEO_E5B 0x20

#define UBX_CFG_GNSS_FLAGS_BEIDOU_B1I 0x01
#define UBX_CFG_GNSS_FLAGS_BEIDOU_B2I 0x10
#define UBX_CFG_GNSS_FLAGS_BEIDOU_B2A 0x80

#define UBX_CFG_GNSS_FLAGS_IMES_L1 0x01

#define UBX_CFG_GNSS_FLAGS_QZSS_L1C_A 0x01
#define UBX_CFG_GNSS_FLAGS_QZSS_L1S   0x04
#define UBX_CFG_GNSS_FLAGS_QZSS_L2C   0x10
#define UBX_CFG_GNSS_FLAGS_QZSS_L5    0x20

#define UBX_CFG_GNSS_FLAGS_GLONASS_L1 0x01
#define UBX_CFG_GNSS_FLAGS_GLONASS_L2 0x10

#define UBX_CFG_NAV5_DYN_MODEL_PORTABLE     0
#define UBX_CFG_NAV5_DYN_MODEL_STATIONARY   2
#define UBX_CFG_NAV5_DYN_MODEL_PEDESTRIAN   3
#define UBX_CFG_NAV5_DYN_MODEL_AUTOMOTIVE   4
#define UBX_CFG_NAV5_DYN_MODEL_SEA          5
#define UBX_CFG_NAV5_DYN_MODEL_AIRBORNE_1G  6
#define UBX_CFG_NAV5_DYN_MODEL_AIRBORNE_2G  7
#define UBX_CFG_NAV5_DYN_MODEL_AIRBORNE_4G  8
#define UBX_CFG_NAV5_DYN_MODEL_WRISTWATCH   9
#define UBX_CFG_NAV5_DYN_MODEL_MOTORBIKE   10
#define UBX_CFG_NAV5_DYN_MODEL_LAWNMOWER   11
#define UBX_CFG_NAV5_DYN_MODEL_SCOOTER     12

#define UBX_CFG_NAV5_FIX_MODE_2D_ONLY    1
#define UBX_CFG_NAV5_FIX_MODE_3D_ONLY    2
#define UBX_CFG_NAV5_FIX_MODE_AUTO_2D_3D 3

#define UBX_CFG_NAV5_UTC_STD_AUTO 0
#define UBX_CFG_NAV5_UTC_STD_USNO 3
#define UBX_CFG_NAV5_UTC_STD_EURO 5
#define UBX_CFG_NAV5_UTC_STD_USSR 6
#define UBX_CFG_NAV5_UTC_STD_NTSC 7
#define UBX_CFG_NAV5_UTC_STD_NPLI 8


/* BITFIELD DEFINITIONS */

/* PRT */

typedef struct __attribute__((packed)) {
  uint16_t en:    1;
  uint16_t pol:   1;
  uint16_t pin:   5;
  uint16_t thres: 9;
} ubx_bitfield_prt_txready_t;

typedef struct __attribute__((packed)) {
  uint32_t :             6;
  uint32_t char_len:     2;

  uint32_t :             1;
  uint32_t parity:       3;
  uint32_t n_stop_bits:  2;
  uint32_t :             2;

  uint32_t :            16;
} ubx_bitfield_prt_mode_t;

typedef struct __attribute__((packed)) {
  uint16_t ubx:   1;
  uint16_t nmea:  1;
  uint16_t rtcm:  1;
  uint16_t :      2;
  uint16_t rtcm3: 1;
  uint16_t :      2;

  uint16_t :      8;
} ubx_bitfield_prt_inprotomask_t;

typedef struct __attribute__((packed)) {
  uint16_t ubx:   1;
  uint16_t nmea:  1;
  uint16_t :      3;
  uint16_t rtcm3: 1;
  uint16_t :      2;

  uint16_t :      8;
} ubx_bitfield_prt_outprotomask_t;

typedef struct __attribute__((packed)) {
  uint16_t :                    1;
  uint16_t extended_tx_timeout: 1;
  uint16_t :                    6;

  uint16_t :                    8;
} ubx_bitfield_prt_flags_t;


/* GNSS */

typedef struct __attribute__((packed)) {
  uint32_t en:           1;
  uint32_t :             7;

  uint32_t :             8;

  uint32_t sig_cfg_mask: 8;

  uint32_t :             8;
} ubx_bitfield_gnss_flags_t;


/* NAV5 */

typedef struct __attribute__((packed)) {
  uint16_t dyn:              1;
  uint16_t min_el:           1;
  uint16_t pos_fix_mode:     1;
  uint16_t dr_lim:           1;
  uint16_t pos_mask:         1;
  uint16_t time_mask:        1;
  uint16_t static_hold_mask: 1;
  uint16_t dgps_mask:        1;

  uint16_t cno_threshold:    1;
  uint16_t :                 1;
  uint16_t utc:              1;
  uint16_t :                 5;
} ubx_bitfield_nav5_mask_t;


/* SBAS */

typedef struct __attribute__((packed)) {
  uint8_t enabled: 1;
  uint8_t test:    1;
  uint8_t :        6;
} ubx_bitfield_sbas_mode_t;

typedef struct __attribute__((packed)) {
  uint8_t range:     1;
  uint8_t diff_corr: 1;
  uint8_t integrity: 1;
  uint8_t :          5;
} ubx_bitfield_sbas_usage_t;


/* PACKET FORMAT DEFINITIONS */

/* UBX-CFG-PRT
 * Change port settings including protocols.
 */
typedef struct __attribute__((packed)) {
  union {
    uint8_t payload[20];
    struct {
      uint8_t port_id;
      uint8_t reserved1;
      ubx_bitfield_prt_txready_t tx_ready;
      ubx_bitfield_prt_mode_t mode;
      uint32_t baud_rate;
      ubx_bitfield_prt_inprotomask_t in_proto_mask;
      ubx_bitfield_prt_outprotomask_t out_proto_mask;
      ubx_bitfield_prt_flags_t flags;
      uint8_t reserved2[2];
    } __attribute__((packed));
  };
} ubx_cfg_prt_t;


/* UBX-CFG-MSG
 * Change rate (or disable) automatic delivery of messages
 * to the current port.
 */
typedef struct __attribute__((packed)) {
  union {
    uint8_t payload[3];
    struct {
      uint8_t msg_class;
      uint8_t msg_id;
      uint8_t rate;
    } __attribute__((packed));
  };
} ubx_cfg_msg_t;


/* UBX-CFG-RATE
 * Change solution rate
 */
typedef struct __attribute__((packed)) {
  union {
    uint8_t payload[6];
    struct {
      uint16_t meas_rate;
      uint16_t nav_rate;
      uint16_t time_ref;
    } __attribute__((packed));
  };
} ubx_cfg_rate_t;


/* UBX-CFG-SBAS
 * SBAS configuration
 */
typedef struct __attribute__((packed)) {
  union {
    uint8_t payload[8];
    struct {
      ubx_bitfield_sbas_mode_t mode;
      ubx_bitfield_sbas_usage_t usage;
      uint8_t max_sbas;
      uint8_t scanmode2;
      uint32_t scanmode1;
    } __attribute__((packed));
  };
} ubx_cfg_sbas_t;


/* UBX-CFG-NAV5
 * Set navigation fix settings.
 */
typedef struct __attribute__((packed)){
  union {
    uint8_t payload[36];
    struct {
      ubx_bitfield_nav5_mask_t mask;
      uint8_t dyn_model;
      uint8_t fix_mode;
      int32_t fixed_alt;
      uint32_t fixed_alt_var;
      int8_t min_elev;
      uint8_t dr_limit;
      uint16_t p_dop, t_dop, p_acc, t_acc;
      uint8_t static_hold_thres;
      uint8_t dgps_timeout;
      uint8_t cno_thresh_num_svs, cno_thresh;
      uint16_t reserved;
      uint16_t static_hold_max_dist;
      uint8_t utc_standard;
      uint8_t reserved2;
      uint32_t reserved3;
    } __attribute__((packed));
  };
} ubx_cfg_nav5_t;


/* UBX_CFG_GNSS
 * GNSS system configuration
 */
 typedef struct __attribute__((packed)) {
   union {
     uint8_t payload[60];
     struct {
      uint8_t msg_ver;
      uint8_t num_trk_ch_hw;
      uint8_t num_trk_ch_use;
      uint8_t num_config_blocks;

      uint8_t gps_gnss_id;
      uint8_t gps_res_trk_ch;
      uint8_t gps_max_trk_ch;
      uint8_t gps_reserved1;
      ubx_bitfield_gnss_flags_t gps_flags;

      uint8_t sbas_gnss_id;
      uint8_t sbas_res_trk_ch;
      uint8_t sbas_max_trk_ch;
      uint8_t sbas_reserved1;
      ubx_bitfield_gnss_flags_t sbas_flags;

      uint8_t galileo_gnss_id;
      uint8_t galileo_res_trk_ch;
      uint8_t galileo_max_trk_ch;
      uint8_t galileo_reserved1;
      ubx_bitfield_gnss_flags_t galileo_flags;

      uint8_t beidou_gnss_id;
      uint8_t beidou_res_trk_ch;
      uint8_t beidou_max_trk_ch;
      uint8_t beidou_reserved1;
      ubx_bitfield_gnss_flags_t beidou_flags;

      uint8_t imes_gnss_id;
      uint8_t imes_res_trk_ch;
      uint8_t imes_max_trk_ch;
      uint8_t imes_reserved1;
      ubx_bitfield_gnss_flags_t imes_flags;

      uint8_t qzss_gnss_id;
      uint8_t qzss_res_trk_ch;
      uint8_t qzss_max_trk_ch;
      uint8_t qzss_reserved1;
      ubx_bitfield_gnss_flags_t qzss_flags;

      uint8_t glonass_gnss_id;
      uint8_t glonass_res_trk_ch;
      uint8_t glonass_max_trk_ch;
      uint8_t glonass_reserved1;
      ubx_bitfield_gnss_flags_t glonass_flags;
     } __attribute__((packed));
   };
 } ubx_cfg_gnss_t;

#endif // CFG_H_
