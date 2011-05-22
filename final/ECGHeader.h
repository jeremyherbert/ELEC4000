#ifndef ECGHEADER_H__
#define ECGHEADER_H__

enum {
  AM_ECGMSG = 10
};

typedef nx_struct ECG_DATA{
  nx_int16_t D1;
  nx_int16_t D2;
  nx_int16_t D3;
  nx_int16_t D4;
  nx_int16_t D5;
  nx_int16_t D6;
  nx_int16_t D7;
  nx_int32_t TIME;
  
} ECG_DATA;

typedef nx_struct ECG_PACKET{
  nx_int16_t NODE_ID;
  nx_int16_t D1;
  nx_int16_t D2;
  nx_int16_t D3;
  nx_int16_t D4;
  nx_int16_t D5;
  nx_int16_t D6;
  nx_int16_t D7;
  nx_int32_t TIME;
  
} ECG_PACKET;

typedef nx_struct UPDATE_PATIENT_PACKET{
  nx_int8_t NODE_ID;
  nx_int16_t READ_INTERVAL;
  nx_int16_t BEAT_INTERVAL;
  nx_int8_t IS_LIVE;
 
} UPDATE_PATIENT_PACKET;


typedef nx_struct REQUEST_MSG{
  nx_int16_t NODE_ID;


} REQUEST_MSG;

typedef nx_struct EMER_MSG{
  nx_int16_t NODE_ID;
  nx_int8_t TYPE;
  nx_int32_t TIME;

} EMER_MSG;



typedef nx_struct REQUEST_DONE_MSG{
  nx_int8_t NODE_ID;
  nx_int32_t TIME;

} REQUEST_DONE_MSG;


typedef nx_struct BEAT_MSG{
  nx_int8_t NODE_ID;

} BEAT_MSG;


//Id's 1, 2 and 3 reserved for alarm
typedef nx_struct ALARM{
  nx_int8_t TYPE;
} ALARM;


#endif //RSSIDEMOMESSAGES_H__
