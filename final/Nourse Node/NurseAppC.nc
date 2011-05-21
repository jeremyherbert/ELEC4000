#include "printf.h"
#include "NurseHeader.h"

configuration NurseAppC {

}
implementation {
  

components NurseC as App;
components  ActiveMessageC, MainC, LedsC;  


  	App.Boot -> MainC;
  	App.Leds -> LedsC;

components new TimerMilliC() as BuzzTimer;
	App.BuzzTimer -> BuzzTimer;

  //components HplMsp430GeneralIOC;
  //App.ADC5 -> HplMsp430GeneralIOC.Port63;

components HplMsp430GeneralIOC as GIO;
	App.io -> GIO.ADC1;

 components UserButtonC;  
 	App.Notify -> UserButtonC;

  components new AMSenderC(AM_ECGMSG) as ECGMsgSender;
  	App.Packet -> ECGMsgSender;
  	App.AMPacket -> ECGMsgSender;
  	App.ECGMsgSend -> ECGMsgSender;
  	App.RadioControl -> ActiveMessageC;
}
