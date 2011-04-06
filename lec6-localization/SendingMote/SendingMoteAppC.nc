#include "RssiDemoMessages.h"

configuration SendingMoteAppC {
} implementation {
  components ActiveMessageC, MainC;  
  components new AMSenderC(AM_RSSIMSG) as RssiMsgSender;
  components new AMReceiverC(AM_RSSIMSG) as RssiMsgReceiver;
  components new AMSenderC(AM_REPORTMSG) as ReportMsgSender;
  components new TimerMilliC() as BeaconTimer;
  components new TimerMilliC() as ReportTimer;
  components CC2420ActiveMessageC;

  components SendingMoteC as App, LedsC;

  App.Boot -> MainC;
  App.Leds -> LedsC;
  App.BeaconTimer -> BeaconTimer;
  App.ReportTimer -> ReportTimer;
  
  App.RssiMsgSend -> RssiMsgSender;
  App.RssiMsgReceive -> RssiMsgReceiver;
  App.ReportMsgSend -> ReportMsgSender;
  App.RadioControl -> ActiveMessageC;
  App -> CC2420ActiveMessageC.CC2420Packet;
}
