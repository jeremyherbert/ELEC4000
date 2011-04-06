#include "RssiDemoMessages.h"
#include "message.h"

configuration RssiBaseAppC {
} implementation {
  components ActiveMessageC, SerialActiveMessageC, MainC;  
  components new SerialAMSenderC(AM_REPORTMATRIXMSG) as ReporMatrixtMsgSender;
  components new AMReceiverC(AM_REPORTMSG) as ReportMsgReceiver;
  components new TimerMilliC() as SendTimer, LedsC;

  components RssiBaseC as App;

  App.Boot -> MainC;
  App.Leds -> LedsC;
  App.SendTimer -> SendTimer;
  
  App.RadioControl -> ActiveMessageC;
  App.SerialControl -> SerialActiveMessageC;
  App.ReportMatrixMsgSend -> ReporMatrixtMsgSender;
  App.ReportMsgReceive -> ReportMsgReceiver;
}

