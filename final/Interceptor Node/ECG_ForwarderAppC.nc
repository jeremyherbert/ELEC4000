#include <6lowpan.h>
#include "message.h"
#include <Timer.h>
#include "ECGHeader.h"

configuration ECG_ForwarderAppC {

} implementation {
  	components MainC, LedsC;
  	components ECG_ForwarderC as App;


	components new TimerMilliC() as Timer0;


	App.Boot -> MainC;
	App.Leds -> LedsC;
	App.TimeKeeper -> Timer0;

	components TimeSyncC;
	TimeSyncC.Boot -> MainC;
	MainC.SoftwareInit -> TimeSyncC;
	App.GlobalTime -> TimeSyncC;


////////////// RADIO ///////////////////////////

	components ActiveMessageC;
	components new AMSenderC(AM_ECGMSG);
	components new AMReceiverC(AM_ECGMSG);

	App.Packet -> AMSenderC;
	App.AMPacket -> AMSenderC;
  	App.ECGMsgSend  -> AMSenderC;
	App.AMControl -> ActiveMessageC;
	App.Receive -> AMReceiverC;





////////////////////////////////////////////



/////////////////UDP/////////////////////////
  components IPDispatchC;

  App.RadioControl -> IPDispatchC;
  components new UdpSocketC() as Status;

  App.Status -> Status;

  components UdpC;


  components UDPShellC;
///////////////////////////////////////////


}
