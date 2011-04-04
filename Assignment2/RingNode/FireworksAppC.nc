#include "Fireworks.h"
#include <Timer.h>

configuration FireworksAppC {
}

implementation {
  components MainC, TimeSyncC;

  MainC.SoftwareInit -> TimeSyncC;
  TimeSyncC.Boot -> MainC;

  components FireworksC as App;
  App.Boot -> MainC;

  //components new AMSenderC(AM_FIREWORKS_MSG);
   //App.AMSend -> AMSenderC;

  components ActiveMessageC;

  App.RadioControl -> ActiveMessageC;
  App.Receive -> ActiveMessageC.Receive[AM_FIREWORKS_MSG];
  App.AMSend -> ActiveMessageC.AMSend[AM_FIREWORKS_MSG];
  App.Packet -> ActiveMessageC;
  App.AMPacket -> ActiveMessageC;
  App.PacketTimeStamp -> ActiveMessageC;

/*
	//////////////////////////////////
	components new AMSenderC(AM_FIREWORKS_MSG);
	components new AMReceiverC(AM_FIREWORKS_MSG);

	App.Packet -> AMSenderC;
	App.AMPacket -> AMSenderC;
  	App.AMSend -> AMSenderC;
	App.AMControl -> ActiveMessageC;
	App.Receive -> AMReceiverC;
	App.PacketTimeStamp -> ActiveMessageC;
*/
///////////////////////////////////////

  components LedsC;
  components new TimerMilliC() as ledTimer;
  App.LedTimer -> ledTimer;


  App.GlobalTime -> TimeSyncC;
  App.TimeSyncInfo -> TimeSyncC;
  App.Leds -> LedsC;

}
