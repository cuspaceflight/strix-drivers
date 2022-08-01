/*
 * uBlox GPS receiver
 * 2014, 2016 Adam Greig, Cambridge University Spaceflight
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "hal.h"
#include "ch.h"
#include "chprintf.h"

#include "ublox.h"
#include "ubx.h"


/* USEFUL MACROS */
#define HEADER_SIZE sizeof(ubx_header_t)
#define CHECKSUM_SIZE sizeof(ubx_checksum_t)

/* thread working area */
static THD_WORKING_AREA(ublox_thd_wa, 512);

/*Serial setup*/
static SerialDriver* ublox_seriald;
static const SerialConfig serial_cfg = {
    .speed = 9600,
    .cr1 = 0,
    .cr2 = USART_CR2_STOP1_BITS,
    .cr3 = 0,
};


/* Config Flag */
static bool gps_configured = false;

//binary_semaphore_t pvt_ready_sem;
ublox_pvt_t pvt_latest;


/* flush serial buffer */
static inline void ublox_flush_buffer(void)
{
  sdReadTimeout(ublox_seriald, NULL, 512, TIME_IMMEDIATE);
  return;
}

/* check for new packets */
static inline int ublox_check_packets(void)
{
  uint8_t sync1, sync2;

  sync1 = sdGet(ublox_seriald);

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "   > > > [SYNC] Sync 1: 0x%02x\r\n", sync1);
#endif

  if (sync1 != UBX_SYNC1) return 0;


  while (1) {
    sync2 = sdGet(ublox_seriald);

#if VERBOSE_DEBUG_GPS
    chprintf(&SD4, "   > > > [SYNC] Sync 2: 0x%x\r\n", sync2);
#endif

    if (sync2 == UBX_SYNC2)      return 1; /* sync successful   */
    else if (sync2 != UBX_SYNC1) return 0; /* sync unsuccessful */
  }

  /* this shoudln't be reached, so if we got here return an "error" */
  /* (ie the same as no sync) */
  return 0;
}

static inline ubx_header_t ublox_get_header(void)
{
  ubx_header_t header;

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, (uint8_t *)"   > > > Waiting for New UBX Packet\r\n");
#endif

  while (!ublox_check_packets());

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, (uint8_t *)"   > > > UBX Packet Found!\r\n");
#endif

  header.class = sdGet(ublox_seriald);
  header.id    = sdGet(ublox_seriald);
  header.length = (((uint16_t)sdGet(ublox_seriald) & 0xFF) << 0) |
                  (((uint16_t)sdGet(ublox_seriald) & 0xFF) << 8);

  return header;
}

static inline int ublox_get_payload(ubx_header_t* header, uint8_t* payload)
{
  ubx_checksum_t checksum;

  /* read in the payload data byte by byte */
  for (uint16_t i = 0; i < header->length; i++) payload[i] = sdGet(ublox_seriald);

  /* calculate the checksum of the data */
  checksum = ubx_calc_checksum(header, payload);

  /* read in the next two bytes and check */
  /* if they match our expected checksum  */
  return (checksum.ck_a == sdGet(ublox_seriald) &&
          checksum.ck_b == sdGet(ublox_seriald));
}

/* Transmit a UBX message over the Serial.
 * Message length is determined from the UBX length field.
 */
static int ublox_send_packet(uint8_t* packet, bool check_ack)
{
  systime_t timeout;
  size_t packet_size, bytes_written;

  ubx_header_t* packet_header;
  ubx_header_t ack_header;

  union {
    ubx_ack_t ack;
    uint8_t raw_data[sizeof(ubx_ack_t)];
  } payload;

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "   > > Beginning Packet Send\r\n");
#endif

  packet_header = (ubx_header_t*) packet;

  packet_size = (size_t) HEADER_SIZE + packet_header->length + CHECKSUM_SIZE;

  /* add checksum */
  ubx_add_checksum(packet);

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "   > > Checksum Added\r\n");
#endif

  /* determine the timeout required to send the packet */
  /* currently assuming 2ms per byte */
  timeout = TIME_MS2I(2 * packet_size);

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "   > > Beginning Write\r\n");
  chprintf(&SD4, "   > > Buffer:\r\n");

  ubx_header_t* header = packet;

  chprintf(&SD4, "   > > [HEADER]   SYNC1 | SYNC2 | CLASS | ID   | LENGTH\r\n");
  chprintf(&SD4, "   > > [HEADER]   0x%02x  | 0x%02x  | 0x%02x  | 0x%02x | 0x%04x\r\n",
           header->sync1, header->sync2, header->class, header->id, header->length);

  size_t offset = header->length + HEADER_SIZE;

  for (size_t i = HEADER_SIZE; i < offset; i++)
    chprintf(&SD4, "   > > [PAYLOAD]  0x%02x\r\n", packet[i]);

  ubx_checksum_t* checksum = packet + offset;
  chprintf(&SD4, "   > > [CHECKSUM] CK_A | CK_B\r\n");
  chprintf(&SD4, "   > > [CHECKSUM] 0x%02x | 0x%02x\r\n", checksum->ck_a, checksum->ck_b);
#endif

  /* write to serial buffer */
  ublox_flush_buffer();
  bytes_written = sdWriteTimeout(ublox_seriald, packet, packet_size, timeout);

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "   > > Write Complete\r\n");
  chprintf(&SD4, "   > > Bytes Written: %d\r\n", bytes_written);
  chprintf(&SD4, "   > > > Header:   %d\r\n", HEADER_SIZE);
  chprintf(&SD4, "   > > > Payload:  %d\r\n", header->length);
  chprintf(&SD4, "   > > > Checksum: %d\r\n", CHECKSUM_SIZE);
  chprintf(&SD4, "   > > Packet Size: %d\r\n", packet_size);
#endif

  if (bytes_written != packet_size) return 0;

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "   > > [PASS] All Bytes Written\r\n");
#endif

  if (!check_ack) return 1;

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "   > > Waiting for Ack...\r\n");
#endif

  /* TODO: implement timeout */
  do {
    ack_header = ublox_get_header();
#if VERBOSE_DEBUG_GPS
    chprintf(&SD4, "   > > [HEADER]   SYNC1 | SYNC2 | CLASS | ID   | LENGTH\r\n");
    chprintf(&SD4, "   > > [HEADER]   0x%02x  | 0x%02x  | 0x%02x  | 0x%02x | 0x%04x\r\n",
             ack_header.sync1, ack_header.sync2, ack_header.class, ack_header.id, ack_header.length);
#endif
  } while (
    !(ack_header.class == UBX_CLASS_ACK &&
      (
          ack_header.id == UBX_ACK_ACK ||
          ack_header.id == UBX_ACK_NAK
      ) &&
      ublox_get_payload(&ack_header, payload.raw_data) &&
      payload.ack.cls_id == packet_header->class &&
      payload.ack.msg_id == packet_header->id)
  );

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "   > > Ack Received\r\n");
#endif

  return (ack_header.id == UBX_ACK_ACK);
}

static inline int ublox_configure_port(void)
/*
 * Enabled only UBX protocol over UART, disable NMEA.
 *
 * Can also be used to configure other options,
 * but note current configuration is hardcoded.
 */
{
  struct __attribute__((packed)) {
    ubx_header_t header;
    ubx_cfg_prt_t config;
    ubx_checksum_t checksum;
  } packet = {
    .header = {
      .sync1  = UBX_SYNC1,
      .sync2  = UBX_SYNC2,
      .class  = UBX_CLASS_CFG,
      .id     = UBX_CFG_PRT,
      .length = (uint16_t) sizeof(ubx_cfg_prt_t)
    },

    .config = {
      .port_id = 0x01,
      .tx_ready = {0},
      .mode = {
        .char_len    = UBX_CONFIG_MODE_CHARLEN_8BIT,      /* 8 bit data */
        .parity      = UBX_CFG_PRT_MODE_PARITY_NO_PARITY, /* no parity  */
        .n_stop_bits = UBX_CFG_PRT_MODE_NSTOPBITS_1       /* 1 stop bit */
      },
      .baud_rate = 9600,
      .in_proto_mask = {
        .ubx   = 1, /* enable UBX protocol */
        .nmea  = 0, /* disable NMEA        */
        .rtcm  = 0, /* disable RTCM2       */
        .rtcm3 = 0  /* disable RTCM3       */
      },
      .out_proto_mask = {
        .ubx   = 1, /* enable UBX protocol */
        .nmea  = 0, /* disable NMEA        */
        .rtcm3 = 0  /* disable RTCM3       */
      },
      .flags = {0}, /* disabled extended timeout */
    }
  };

  return ublox_send_packet((uint8_t*) &packet, false);
}

static inline int ublox_configure_gnss(void)
{
  struct __attribute__((packed)) {
    ubx_header_t header;
    ubx_cfg_gnss_t config;
    ubx_checksum_t checksum;
  } packet = {
    .header = {
      .sync1  = UBX_SYNC1,
      .sync2  = UBX_SYNC2,
      .class  = UBX_CLASS_CFG,
      .id     = UBX_CFG_GNSS,
      .length = (uint16_t) sizeof(ubx_cfg_gnss_t)
    },

    .config = {
      .msg_ver = 0x00,
      .num_trk_ch_hw = 32,  /* FIXME: are these read-only? */
      .num_trk_ch_use = 32, /* ^^^^^^^^^^^^^^^^^^^^^^^^^^^ */
      .num_config_blocks = 7,

      /* CONFIG BLOCK 1: GPS  */
      /* enable L1C/A and L2C */
      .gps_gnss_id = UBX_GNSS_ID_GPS,
      .gps_res_trk_ch = 31,
      .gps_max_trk_ch = 31,
      .gps_flags = {
        .en = 1,
        .sig_cfg_mask = (
            UBX_CFG_GNSS_FLAGS_GPS_L1C_A |
            UBX_CFG_GNSS_FLAGS_GPS_L2C
        )
      },

      /* CONFIG BLOCK 2: SBAS */
      /* disable */
      .sbas_gnss_id = UBX_GNSS_ID_SBAS,

      /* CONFIG BLOCK 3: Galileo */
      /* disable                 */

      /* CONFIG BLOCK 4: BeiDou */
      /* disable                */
      .beidou_gnss_id = UBX_GNSS_ID_BEIDOU,

      /* CONFIG BLOCK 5: IMES */
      /* disable              */
      .imes_gnss_id = UBX_GNSS_ID_IMES,

      /* CONFIG BLOCK 6: QZSS */
      /* enable L1C/A, L2C    */
      .qzss_gnss_id = UBX_GNSS_ID_QZSS,
      .qzss_res_trk_ch = 1,
      .qzss_max_trk_ch = 1,
      .qzss_flags = {
         .en = 1,
         .sig_cfg_mask = (
             UBX_CFG_GNSS_FLAGS_QZSS_L1C_A |
             UBX_CFG_GNSS_FLAGS_QZSS_L2C
         )
      },

      /* CONFIG BLOCK 7: GLONASS */
      /* disable                 */
      .glonass_gnss_id = UBX_GNSS_ID_GLONASS,
    }
  };

  return ublox_send_packet((uint8_t*) &packet, true);
}

static inline int ublox_configure_navigation(void)
{
  struct __attribute__((packed)) {
    ubx_header_t header;
    ubx_cfg_nav5_t config;
    ubx_checksum_t checksum;
  } packet = {
    .header = {
      .sync1  = UBX_SYNC1,
      .sync2  = UBX_SYNC2,
      .class  = UBX_CLASS_CFG,
      .id     = UBX_CFG_NAV5,
      .length = (uint16_t) sizeof(ubx_cfg_nav5_t)
    },

    .config = {
      .mask = {
        .dyn              = 1,
        .min_el           = 0,
        .pos_fix_mode     = 0,
        .dr_lim           = 0,
        .pos_mask         = 0,
        .time_mask        = 0,
        .static_hold_mask = 0,
        .dgps_mask        = 0,
        .cno_threshold    = 0,
        .utc              = 1
      },
      .dyn_model = UBX_CFG_NAV5_DYN_MODEL_AIRBORNE_4G,
      .utc_standard = UBX_CFG_NAV5_UTC_STD_USNO
    }
  };

  return ublox_send_packet((uint8_t*) &packet, true);
}

static inline int ublox_configure_sbas(void)
{
  struct __attribute__((packed)) {
      ubx_header_t header;
      ubx_cfg_sbas_t config;
      ubx_checksum_t checksum;
  } packet = {
    .header = {
      .sync1  = UBX_SYNC1,
      .sync2  = UBX_SYNC2,
      .class  = UBX_CLASS_CFG,
      .id     = UBX_CFG_SBAS,
      .length = (uint16_t) sizeof(ubx_cfg_sbas_t)
    },

    .config = {
      .mode = {0} /* DISABLE SBAS */
    }
  };

  return ublox_send_packet((uint8_t*) &packet, true);
}

static inline int ublox_configure_posecef(void)
{
  struct __attribute__((packed)) {
        ubx_header_t header;
        ubx_cfg_msg_t config;
        ubx_checksum_t checksum;
    } packet = {

      .header = {
        .sync1  = UBX_SYNC1,
        .sync2  = UBX_SYNC2,
        .class  = UBX_CLASS_CFG,
        .id     = UBX_CFG_MSG,
        .length = (uint16_t) sizeof(ubx_cfg_msg_t)
      },

      .config = {
        .msg_class = UBX_CLASS_NAV,
        .msg_id = UBX_NAV_POSECEF,
        .rate = 0x01
      }
    };

    return ublox_send_packet((uint8_t*) &packet, true);
}

static THD_FUNCTION(ublox_thd, arg)
{
    (void)arg;
    chRegSetThreadName("GPS");

    ubx_header_t header;
//    char buf[100];

    union {
      uint8_t raw_data[
        sizeof(
            union {
              ubx_nav_posecef_t nav_posecef;
              ubx_nav_pvt_t nav_pvt;
            }
        )
      ];

      ubx_nav_posecef_t posecef;
      ubx_nav_pvt_t pvt;
    } payload;

    if (!gps_configured) return;

    while(1) {
      header = ublox_get_header();

      if (header.class == UBX_CLASS_NAV)
        /* only this is implemented for now */
      {
        switch (header.id) {
          case UBX_NAV_POSECEF:
            ublox_get_payload(&header, payload.raw_data);

//            snprintf(
//              buf, 100,
//
//              "POSECEF Packet:    \r\n"
//              "  Time of Week: %i \r\n"
//              "  Lon: %l, Lat: %l \r\n"
//              "  Height: %l       \r\n"
//              "  Accuracy: %l     \r\n",
//
//              payload.posecef.i_tow,
//              payload.posecef.lon,
//              payload.posecef.lat,
//              payload.posecef.height,
//              payload.posecef.p_acc
//            );

//            sdWrite(&SD4, (uint8_t *)buf, 100);
            sdWrite(&SD4, (uint8_t *)"POSECEF packet.\r\n", 16);
            break;
          case UBX_NAV_PVT:
            sdWrite(&SD4, (uint8_t *)"PVT packet.\r\n", 12);
            break;
          default:
            sdWrite(&SD4, (uint8_t *)"Unknown packet.\r\n", 16);
            break;
        }
      }
    }
}

void ublox_reset(void)
{
#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "...Resetting U-blox Hardware: ");
#endif

  /* Toggle GPS reset line according to the following profile: */
  /*                                                           */
  /* ....  /- 300 ms -\ /-- 500ms --\  ....                    */
  /* _____              ___________________                    */
  /*      |____________|                     [GPS_RESETn]      */
  /*                                                           */

  /* set line low for 300 ms */
  palClearLine(LINE_GPS_RESET);
  chThdSleepMilliseconds(300);

  /* set line high for 500 ms */
  palSetLine(LINE_GPS_RESET);
  chThdSleepMilliseconds(500);

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "Complete!\r\n");
#endif
}

void ublox_init(SerialDriver* seriald)
{
#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "Initialising U-blox (GPS) Module...\r\n");
#endif

  ublox_seriald = seriald;

  /* We'll reset the uBlox so it's in a known state */
  ublox_reset();
#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "...Starting Serial Interface: ");
#endif

  sdStart(ublox_seriald, &serial_cfg);

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "Complete!\r\n");
  chprintf(&SD4, "...Waiting for GPS Configuration (blocking operation):\r\n");
#endif

  union {
    struct {
      uint8_t port:       1;
      uint8_t gnss:       1;
      uint8_t navigation: 1;
      uint8_t sbas:       1;
      uint8_t posecef:    1;
    } __attribute__((packed));
    uint8_t byte_val;
  } config_flags;

  config_flags.byte_val = 0x00;

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "   > Port\r\n");
#endif
  config_flags.port = ublox_configure_port();

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "   > GNSS\r\n");
#endif
  config_flags.gnss = ublox_configure_gnss();

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "   > Navigation\r\n");
#endif
  config_flags.navigation = ublox_configure_navigation();

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "   > SBAS\r\n");
#endif
  config_flags.sbas = ublox_configure_sbas();

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "   > POSECEF\r\n");
#endif
  config_flags.posecef = ublox_configure_posecef();

  if (config_flags.byte_val == 0x00) {
    gps_configured = true;
#if VERBOSE_DEBUG_GPS
    chprintf(&SD4, "   > GPS Successfully Configured!\r\n");
  } else {
    chprintf(&SD4, "   > GPS Configuration Unsuccessful.\r\n");

    chprintf(&SD4,
        config_flags.port ? "   > > Port: [PASS]\r\n" :
                            "   > > Port: [FAIL]\r\n"
    );

    chprintf(&SD4,
        config_flags.gnss ? "   > > GNSS: [PASS]\r\n" :
                            "   > > GNSS: [FAIL]\r\n"
    );

    chprintf(&SD4,
        config_flags.navigation ? "   > > Navigation: [PASS]\r\n" :
                                  "   > > Navigation: [FAIL]\r\n"
    );

    chprintf(&SD4,
        config_flags.sbas ? "   > > SBAS: [PASS]\r\n" :
                            "   > > SBAS: [FAIL]\r\n"
    );

    chprintf(&SD4,
        config_flags.posecef ? "   > > POSECEF: [PASS]\r\n" :
                               "   > > POSECEF: [FAIL]\r\n"
    );
#endif
  }

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "U-blox (GPS) Module Init Complete!\r\n\r\n");
#endif

  return;
}

/* Init GPS Thread */
void ublox_thd_init(void)
{
#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "Initialising U-blox (GPS) Thread...\r\n");
#endif

  chThdCreateStatic(ublox_thd_wa, sizeof(ublox_thd_wa), NORMALPRIO, ublox_thd, NULL);

#if VERBOSE_DEBUG_GPS
  chprintf(&SD4, "U-blox (GPS) Thread Init Complete!\r\n\r\n");
#endif

  return;
}
