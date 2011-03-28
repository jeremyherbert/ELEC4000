#include "RssiMessage.h"
#include <Timer.h>

configuration SendingMoteAppC {
} implementation {
  components ActiveMessageC, MainC, LedsC;  
  components new AMSenderC(AM_RSSIMSG) as RssiMsgSender;
  components new TimerMilliC() as SendTimer;

  components SendingMoteC as App;

  App.Boot -> MainC;
  App.Leds -> LedsC;
  App.SendTimer -> SendTimer;
  App.Packet -> RssiMsgSender;
  App.AMPacket -> RssiMsgSender;
  App.RssiMsgSend -> RssiMsgSender;
  App.RadioControl -> ActiveMessageC;
}
