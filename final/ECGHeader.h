#ifndef ECGHEADER_H__
#define ECGHEADER_H__

enum {
  AM_ECGMSG = 10
};

typedef nx_struct ECG_DATA{
  nx_int8_t NODE_ID;
  nx_int8_t D1;
  nx_int8_t D2;
  nx_int8_t D3;
  nx_int8_t D4;
  nx_int8_t D5;
  nx_int8_t D6;
  nx_int16_t D7;
  nx_int8_t TIME;
  
} ECG_DATA;

typedef nx_struct BEAT_MSG{
  nx_int8_t NODE_ID;


} BEAT_MSG;

#endif //RSSIDEMOMESSAGES_H__
