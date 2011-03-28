#include <6lowpan.h>
#include "message.h"
#include <Timer.h>
//#include "RSSI_Forwarder.h"
#include "RssiMessage.h"

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




  components CC2420ActiveMessageC;
  App -> CC2420ActiveMessageC.CC2420Packet;

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
