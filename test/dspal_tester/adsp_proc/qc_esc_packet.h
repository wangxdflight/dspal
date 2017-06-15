/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */
#ifndef QC_ESC_PACKET
#define QC_ESC_PACKET

#ifdef __cplusplus
extern "C" {
#endif

#include "crc16.h"
#include <stdint.h>

int32_t qc_esc_create_packet(uint8_t type, uint8_t * data, uint16_t size, uint8_t * out, uint16_t out_size);

int32_t qc_esc_create_version_request_packet(uint8_t id, uint8_t * out, uint16_t out_size);
int32_t qc_esc_create_reset_packet(uint8_t id, uint8_t * out, uint16_t out_size);
int32_t qc_esc_create_sound_packet(uint8_t frequency, uint8_t duration, uint8_t power, uint8_t mask,uint8_t * out, uint16_t out_size);

int32_t qc_esc_create_pwm_packet4(int16_t pwm0, int16_t pwm1, int16_t pwm2, int16_t pwm3,
                                  uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3,
                                  uint8_t * out, uint16_t out_size);

int32_t qc_esc_create_pwm_packet4_fb(int16_t pwm0, int16_t pwm1, int16_t pwm2, int16_t pwm3,
                                     uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3,
                                     int32_t fb_id, uint8_t * out, uint16_t out_size);

int32_t qc_esc_create_rpm_packet4(int16_t rpm0, int16_t rpm1, int16_t rpm2, int16_t rpm3,
                                  uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3,
                                  uint8_t * out, uint16_t out_size);

int32_t qc_esc_create_rpm_packet4_fb(int16_t rpm0, int16_t rpm1, int16_t rpm2, int16_t rpm3,
                                     uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3,
                                     int32_t fb_id, uint8_t * out, uint16_t out_size);


#define ESC_PACKET_HEADER    0xAF

enum { ESC_ERROR_BAD_CHECKSUM = -1,
       ESC_NO_PACKET
};

enum { ESC_PACKET_POS_HEADER1 = 0,
       ESC_PACKET_POS_LENGTH,
       ESC_PACKET_POS_TYPE,
       ESC_PACKET_POS_DATA };

//buffer packet definition
typedef struct
{
  uint8_t  len_received; //number of chars received so far
  uint8_t  len_expected; //expected number of chars based on header
  uint8_t  * bp;        //pointer to the next write position in the buffer
  uint16_t crc;
  uint8_t  buffer[32];  //buffer for data
} EscPacket;


typedef struct
{
  uint8_t  header;
  uint8_t  length;
  uint8_t  type;

  uint8_t  id;                 //0..8 hardcoded, 127=use pin selection
  uint16_t sw_version;
  uint16_t hw_version;

  uint32_t unique_id;
  uint16_t crc;
}  __attribute__ ((__packed__)) QC_ESC_VERSION_INFO;


//feed one char and see if we have accumulated a complete packet
int16_t   EscPacketProcessChar(uint8_t c, EscPacket * packet);

//get a pointer to the packet type
static inline uint8_t  EscPacketGetType(EscPacket * packet)   { return packet->buffer[ESC_PACKET_POS_TYPE];    }
static inline uint8_t  EscPacketRawGetType(uint8_t * packet)  { return packet[ESC_PACKET_POS_TYPE];            }

//get a pointer to the packet payload
static inline uint8_t * EscPacketGetData(EscPacket * packet)  { return &(packet->buffer[ESC_PACKET_POS_DATA]); }
static inline uint8_t * EscPacketRawGetData(uint8_t * packet) { return &(packet[ESC_PACKET_POS_DATA]);         }

static inline uint8_t   EscPacketGetSize(EscPacket * packet)  { return packet->buffer[ESC_PACKET_POS_LENGTH];  }


//calculate the checksum of a data array
static inline uint16_t EscPacketChecksum(uint8_t * buf, uint16_t size)
{
  uint16_t crc = crc16_init();
  return crc16(crc, buf, size);
}

static inline uint16_t EscPacketChecksumGet(EscPacket * packet)
{
  return packet->crc;
}

static inline void EscPacketChecksumReset(EscPacket * packet)
{
  packet->crc = crc16_init();
}

static inline void EscPacketChecksumProcessChar(EscPacket * packet, uint8_t c)
{
  packet->crc = crc16_byte(packet->crc, c);
}


//initialize the packet
static inline void EscPacketInit(EscPacket * packet)
{
  packet->len_received = 0;
  packet->len_expected = 0;
  packet->bp           = 0;

  EscPacketChecksumReset(packet);
}

static inline void EscPacketReset(EscPacket * packet)
{
  packet->len_received = 0;
}

#endif //QC_ESC_PACKET

#ifdef __cplusplus
}
#endif
