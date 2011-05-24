#include "../ECGHeader.h"
#include <Timer.h>
#include "StorageVolumes.h"

configuration patientAppC {
} implementation {
  
	components patientC as App;
	
	components  MainC, LedsC; 
		App.Boot -> MainC;
	  	App.Leds -> LedsC; 

  	components new TimerMilliC() as ReadTimer;
  	components new TimerMilliC() as HeartBeatTimer;
	  	App.ReadTimer -> ReadTimer;
	  	App.HeartBeatTimer -> HeartBeatTimer;
  
  
  	components ActiveMessageC;
  	components new AMSenderC(AM_ECGMSG) as ECGMsgSender;
  	components new AMReceiverC(AM_ECGMSG);
	  	App.Packet -> ECGMsgSender;
	  	App.AMPacket -> ECGMsgSender;
	  	App.ECGMsgSend -> ECGMsgSender;
	  	App.RadioControl -> ActiveMessageC;
	  	App.Receive -> AMReceiverC;


  	components new LogStorageC(VOLUME_LOGTEST, TRUE);
	  	App.LogRead -> LogStorageC;
	  	App.LogWrite -> LogStorageC;


  	components UserButtonC;  
	  	App.Notify -> UserButtonC;

	
  	components TimeSyncC;
	  	TimeSyncC.Boot -> MainC;
	  	MainC.SoftwareInit -> TimeSyncC;
	  	App.GlobalTime -> TimeSyncC;

	// ECG data stuff
		components new AdcReadClientC(), new TimerMilliC() as RTimer, new TimerMilliC() as MovementDelayTimer, new TimerMilliC() as AlarmTimer, new TimerMilliC() as SampleTimer, new TimerMilliC() as HRTimer;
			App.VoltageRead -> AdcReadClientC;
			AdcReadClientC.AdcConfigure -> App.VoltageConfigure;
			App.RTimer -> RTimer;
			App.MovementDelayTimer -> MovementDelayTimer;
		  App.AlarmTimer -> AlarmTimer;
		  App.SampleTimer -> SampleTimer;
			App.HRTimer -> HRTimer;
			

}
