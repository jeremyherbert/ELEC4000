#include <6lowpan.h>
#include "message.h"
#include <Timer.h>
#include "RSSI_Forwarder.h"

configuration RSSI_ForwarderC {

} implementation {
  	components MainC, LedsC;
  	components RSSI_ForwarderP as App;


	components new TimerMilliC() as Timer0;
	components new TimerMilliC() as Timer1;

	App.Boot -> MainC;
	App.Leds -> LedsC;
	App.Timer0 -> Timer0;


////////////// RADIO ///////////////////////////

	components ActiveMessageC;
	components new AMSenderC(AM_RSSIMSG);
	components new AMReceiverC(AM_RSSIMSG);

	App.Packet -> AMSenderC;
	App.AMPacket -> AMSenderC;
  	App.AMSend -> AMSenderC;
	App.AMControl -> ActiveMessageC;
	App.Receive -> AMReceiverC;



#ifdef __CC2420_H__
  components CC2420ActiveMessageC;
  App -> CC2420ActiveMessageC.CC2420Packet;
#elif  defined(PLATFORM_IRIS)
  components  RF2xxActiveMessageC;
  App -> RF2xxActiveMessageC.PacketRSSI;
#elif defined(TDA5250_MESSAGE_H)
  components Tda5250ActiveMessageC;
  App -> Tda5250ActiveMessageC.Tda5250Packet;
#endif
////////////////////////////////////////////



/////////////////UDP/////////////////////////
  components IPDispatchC;

  App.RadioControl -> IPDispatchC;
  components new UdpSocketC() as Status;

  App.Status -> Status;

  App.StatusTimer -> Timer1;

  components UdpC;


  components UDPShellC;
///////////////////////////////////////////


}
