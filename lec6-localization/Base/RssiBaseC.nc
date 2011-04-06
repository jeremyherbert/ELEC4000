#include "RssiDemoMessages.h"  

module RssiBaseC {
  uses interface Boot;
  uses interface Leds;
  uses interface SplitControl as RadioControl;
  uses interface SplitControl as SerialControl;
  uses interface Timer<TMilli> as SendTimer;
  
  uses interface AMSend as ReportMatrixMsgSend;
  uses interface Receive as ReportMsgReceive;
} implementation {
  message_t report_msg;
  ReportMatrixMsg *report_data;

  void initReport() {
    uint8_t i,j;
    for (i=0; i<NUM_NODES; i++)
      for (j=0; j<NUM_NODES; j++)
        report_data->rssi[i][j]=UNDEF_RSSI;
  }

  event void Boot.booted(){
    call RadioControl.start();
    call SerialControl.start();
    report_data = call ReportMatrixMsgSend.getPayload(&report_msg, sizeof(ReportMatrixMsg)); 
    initReport();
  }

  event void RadioControl.startDone(error_t result){
    call SendTimer.startPeriodic(SEND_REPORT_MS);
  }
  event void RadioControl.stopDone(error_t result){}

  event void SerialControl.startDone(error_t result){}
  event void SerialControl.stopDone(error_t result){}

  event message_t* ReportMsgReceive.receive(message_t* msg, void* payload, uint8_t len) {
    uint8_t i;
    ReportMsg *rdata = (ReportMsg *)payload;
#ifdef PRINTF
      printf("report from %d: ", rdata->id);
#endif      
    for (i=0; i<NUM_NODES; i++){
      report_data->rssi[rdata->id][i]=rdata->rssi[i];
#ifdef PRINTF
      printf(" %d%c", rdata->rssi[i], i==NUM_NODES-1?'\n':',');
#endif      
    }
#ifdef PRINTF
    printfflush();
#endif      
    call Leds.led1Toggle();
    return msg;
  }

  event void SendTimer.fired(){
    call Leds.led0Toggle();
    call ReportMatrixMsgSend.send(AM_BROADCAST_ADDR, &report_msg, sizeof(ReportMatrixMsg));    
  }
  event void ReportMatrixMsgSend.sendDone(message_t *m, error_t error){}
}
