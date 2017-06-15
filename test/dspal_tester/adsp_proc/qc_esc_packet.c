/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */
#include "crc16.h"
#include "qc_esc_packet.h"
#include <string.h>


int32_t qc_esc_create_version_request_packet(uint8_t id, uint8_t * out, uint16_t out_size)
{
  return qc_esc_create_packet(0, &id, 1, out, out_size);
}

int32_t qc_esc_create_reset_packet(uint8_t id, uint8_t * out, uint16_t out_size)
{
  char payload[]  = "RESET0";
  payload[5]      += id;

  return qc_esc_create_packet(10, (uint8_t*)payload, sizeof(payload), out, out_size);
}


int32_t qc_esc_create_sound_packet(uint8_t frequency, uint8_t duration, uint8_t power, uint8_t mask,uint8_t * out, uint16_t out_size)
{
  uint8_t data[4] = {frequency,duration,power,mask};
  return qc_esc_create_packet(3, (uint8_t*)&(data[0]), 4, out, out_size);
}

int32_t qc_esc_create_pwm_packet4(int16_t pwm0, int16_t pwm1, int16_t pwm2, int16_t pwm3,
                                  uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3,
                                  uint8_t * out, uint16_t out_size)
{
  return qc_esc_create_pwm_packet4_fb(pwm0,pwm1,pwm2,pwm3,led0,led1,led2,led3,-1,out,out_size);
}

int32_t qc_esc_create_pwm_packet4_fb(int16_t pwm0, int16_t pwm1, int16_t pwm2, int16_t pwm3,
                                     uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3,
                                     int32_t fb_id, uint8_t * out, uint16_t out_size)
{
  uint16_t data[5];
  uint16_t leds = 0;

  if (fb_id != -1) fb_id = fb_id % 4;

  //limit the pwm commands
  if (pwm0 > 800) pwm0 = 800; if (pwm0 < -800) pwm0 = -800;
  if (pwm1 > 800) pwm1 = 800; if (pwm1 < -800) pwm1 = -800;
  if (pwm2 > 800) pwm2 = 800; if (pwm2 < -800) pwm2 = -800;
  if (pwm3 > 800) pwm3 = 800; if (pwm3 < -800) pwm3 = -800;

  //least significant bit is used for feedback request
  pwm0 &= ~(0x0001); pwm1 &= ~(0x0001); pwm2 &= ~(0x0001); pwm3 &= ~(0x0001);

  if (fb_id == 0) pwm0 |= 0x0001; if (fb_id == 1) pwm1 |= 0x0001;
  if (fb_id == 2) pwm2 |= 0x0001; if (fb_id == 3) pwm3 |= 0x0001;

  leds |=             led0 & 0b00000111;
  leds |=            (led1 & 0b00000111)  << 3;
  leds |= ((uint16_t)(led2 & 0b00000111)) << 6;
  leds |= ((uint16_t)(led3 & 0b00000111)) << 9;

  data[0] = pwm0; data[1] = pwm1; data[2] = pwm2; data[3] = pwm3; data[4] = leds;
  return qc_esc_create_packet(1, (uint8_t*)&(data[0]), 10, out, out_size);
}


int32_t qc_esc_create_rpm_packet4(int16_t rpm0, int16_t rpm1, int16_t rpm2, int16_t rpm3,
                                  uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3,
                                  uint8_t * out, uint16_t out_size)
{
  return qc_esc_create_rpm_packet4_fb(rpm0,rpm1,rpm2,rpm3,led0,led1,led2,led3,-1,out,out_size);
}

int32_t qc_esc_create_rpm_packet4_fb(int16_t rpm0, int16_t rpm1, int16_t rpm2, int16_t rpm3,
                                     uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3,
                                     int32_t fb_id, uint8_t * out, uint16_t out_size)
{
  uint16_t data[5];
  uint16_t leds = 0;

  if (fb_id != -1) fb_id = fb_id % 4;

  //least significant bit is used for feedback request
  rpm0 &= ~(0x0001); rpm1 &= ~(0x0001); rpm2 &= ~(0x0001); rpm3 &= ~(0x0001);

  if (fb_id == 0) rpm0 |= 0x0001; if (fb_id == 1) rpm1 |= 0x0001;
  if (fb_id == 2) rpm2 |= 0x0001; if (fb_id == 3) rpm3 |= 0x0001;

  leds |=             led0 & 0b00000111;
  leds |=            (led1 & 0b00000111)  << 3;
  leds |= ((uint16_t)(led2 & 0b00000111)) << 6;
  leds |= ((uint16_t)(led3 & 0b00000111)) << 9;

  data[0] = rpm0; data[1] = rpm1; data[2] = rpm2; data[3] = rpm3; data[4] = leds;
  return qc_esc_create_packet(2, (uint8_t*)&(data[0]), 10, out, out_size);
}

int32_t qc_esc_create_packet(uint8_t type, uint8_t * data, uint16_t size, uint8_t * out, uint16_t out_size)
{
  uint16_t packet_size = size + 5;

  if (packet_size > 255)      return -1;
  if (out_size < packet_size) return -2;

  out[0] = 0xAF;
  out[1] = packet_size;
  out[2] = type;

  memcpy(&(out[3]),data,size);

  uint16_t crc = crc16_init();
  crc = crc16(crc, &(out[1]), packet_size-3);

  memcpy(&(out[packet_size-2]),&crc,sizeof(uint16_t));

  return packet_size;
}




//feed in a character and see if we got a complete packet
int16_t   EscPacketProcessChar(uint8_t c, EscPacket * packet)
{
  int16_t ret = ESC_NO_PACKET;

  uint16_t chk_comp;
  uint16_t chk_rcvd;
  
  //packet->len_expected = 255;  //FIXME: for testing only
  
  if (packet->len_received >= (sizeof(packet->buffer)-1))
  {
    packet->len_received = 0;
  }

  switch (packet->len_received)
  {
    case 0:  //header
      packet->bp = packet->buffer;      //reset the pointer for storing data
      EscPacketChecksumReset(packet);         //reset the checksum to starting value

      if (c != ESC_PACKET_HEADER)       //check the packet header
      {
        packet->len_received = 0;
        ret = -1;
        break;
      }

      packet->len_received++;
      *(packet->bp)++ = c;
      break;

    case 1:  //length
      packet->len_received++;
      *(packet->bp)++      = c;
      packet->len_expected = c;
      
      if (packet->len_expected >= (sizeof(packet->buffer)-1))
      {
        packet->len_received = 0;
        ret = -1;
        break;
      }
      
      EscPacketChecksumProcessChar(packet,c);
      break;

    default: //rest of the packet
      packet->len_received++;
      *(packet->bp)++ = c;

      if (packet->len_received < (packet->len_expected-1)) //do not compute checksum of checksum (last 2 bytes)
        EscPacketChecksumProcessChar(packet,c);

      if (packet->len_received < packet->len_expected)     //waiting for more bytes
      {
        break;
      }

      //grab the computed checksum and compare against the received value
      chk_comp = EscPacketChecksumGet(packet);

      memcpy(&chk_rcvd, packet->bp-2, sizeof(uint16_t));

      if (chk_comp == chk_rcvd) ret  = packet->len_received;
      else                      ret  = ESC_ERROR_BAD_CHECKSUM;

      packet->len_received = 0;
      break;
  }

  return ret;
}
