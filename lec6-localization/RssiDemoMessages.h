#ifndef RSSIDEMOMESSAGES_H__
#define RSSIDEMOMESSAGES_H__

enum {
  NUM_NODES = 5,
  SEND_BEACON_MS = 200,
  SEND_REPORT_MS = 1000,
  UNDEF_RSSI = 127,
  BASE_ID = 100,
  AM_RSSIMSG = 10,
  AM_REPORTMSG = 11,
  AM_REPORTMATRIXMSG = 12,
};

typedef nx_struct RssiMsg{
  nx_uint8_t id;
} RssiMsg;

typedef nx_struct ReportMsg{
  nx_uint8_t id;
  nx_int8_t rssi[NUM_NODES];
} ReportMsg;

typedef nx_struct ReportMatrixMsg{
  nx_int8_t rssi[NUM_NODES][NUM_NODES];
} ReportMatrixMsg;

#endif //RSSIDEMOMESSAGES_H__
