#include "ECGHeader.h"
#include <Timer.h>
#include "StorageVolumes.h"

configuration patientAppC {
} implementation {
  components ActiveMessageC, MainC, LedsC;  
  components new AMSenderC(AM_ECGMSG) as ECGMsgSender;

  components new TimerMilliC() as ReadTimer;
  components new TimerMilliC() as HeartBeatTimer;

  components patientC as App;

  App.Boot -> MainC;
  App.Leds -> LedsC;

  App.ReadTimer -> ReadTimer;
  App.HeartBeatTimer -> HeartBeatTimer;
  App.Packet -> ECGMsgSender;
  App.AMPacket -> ECGMsgSender;
  App.ECGMsgSend -> ECGMsgSender;
  App.RadioControl -> ActiveMessageC;


  components new LogStorageC(VOLUME_LOGTEST, TRUE);
  App.LogRead -> LogStorageC;
  App.LogWrite -> LogStorageC;


  components UserButtonC;  
  App.Notify -> UserButtonC;

	
  components TimeSyncC;
  TimeSyncC.Boot -> MainC;
  MainC.SoftwareInit -> TimeSyncC;
  App.GlobalTime -> TimeSyncC;


}
