#include "RssiDemoMessages.h"

module SendingMoteC {
  uses interface Boot;
  uses interface Leds;
  uses interface Timer<TMilli> as ReportTimer;
  uses interface Timer<TMilli> as BeaconTimer;
  
  uses interface AMSend as RssiMsgSend;
  uses interface Receive as RssiMsgReceive;
  uses interface AMSend as ReportMsgSend;
  uses interface SplitControl as RadioControl;
  uses interface CC2420Packet;
} implementation {
  message_t rssi_msg;
  RssiMsg *rssi_msg_data;
  message_t report_msg;
  ReportMsg *report_msg_data;

  typedef struct neighbor_t{
    int8_t rssi;
    uint8_t num;
  } neighbor_t;

  neighbor_t ntable[NUM_NODES];

  void initNTable() {
    uint8_t i=0;
    for (i=0; i<NUM_NODES; i++)
      ntable[i].rssi=0;
      ntable[i].num=0;
  }

  event void Boot.booted(){
    call RadioControl.start();
    rssi_msg_data = call RssiMsgSend.getPayload(&rssi_msg, sizeof(RssiMsg)); 
    rssi_msg_data->id = TOS_NODE_ID;
    report_msg_data = call ReportMsgSend.getPayload(&report_msg, sizeof(ReportMsg)); 
    report_msg_data->id = TOS_NODE_ID;
    initNTable();
  }

  event void RadioControl.startDone(error_t result){
    call BeaconTimer.startPeriodic(SEND_BEACON_MS);
    call ReportTimer.startPeriodic(SEND_REPORT_MS);
  }

  event void RadioControl.stopDone(error_t result){}

  task void sendReport();

  event void BeaconTimer.fired(){
    call RssiMsgSend.send(AM_BROADCAST_ADDR, &rssi_msg, sizeof(RssiMsg));    
  }
  event void ReportTimer.fired(){
    post sendReport();
  }
  event void RssiMsgSend.sendDone(message_t *m, error_t error){}

  event message_t* RssiMsgReceive.receive(message_t* msg, void* payload, uint8_t len) {
    RssiMsg *rdata = (RssiMsg *)payload;
    ntable[rdata->id].rssi= call CC2420Packet.getRssi(msg)-45;
    ntable[rdata->id].num++;
    return msg;
  }

  uint8_t sending=0;
  task void sendReport() {
    uint8_t i=0;
    for(i=0; i<NUM_NODES; i++) {
      if(ntable[i].num==0)
        report_msg_data->rssi[i]=UNDEF_RSSI;
      else
        report_msg_data->rssi[i]=ntable[i].rssi;
    }
    initNTable();

    if (!sending) {
      if(call ReportMsgSend.send(BASE_ID, &report_msg, sizeof(ReportMsg))==SUCCESS)
        sending = 1;
    }
  }
  event void ReportMsgSend.sendDone(message_t *m, error_t error){
    sending=0;
  }
}
