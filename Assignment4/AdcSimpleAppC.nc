#include "clap.h"

configuration AdcSimpleAppC {
}
implementation {
  components MainC, AdcSimpleC as App, new AdcReadClientC(), LedsC, new TimerMilliC() as timer, TimeSyncC;
  //components new SensirionSht11C() as HumidityTempC;

  MainC.SoftwareInit -> TimeSyncC;
  TimeSyncC.Boot -> MainC;


  App.Boot -> MainC;
  App.VoltageRead -> AdcReadClientC;
  // Uncomment the following line when using SensirionSht11C;
  // AdcSimpleC.VoltageRead -> HumidityTempC.Temperature;
  AdcReadClientC.AdcConfigure -> App.VoltageConfigure;
  App.Leds -> LedsC;
  App.Timer -> timer;


  
  components ActiveMessageC;

  App.RadioControl -> ActiveMessageC;
  App.Receive -> ActiveMessageC.Receive[AM_CLAP_MSG];
  App.AMSend -> ActiveMessageC.AMSend[AM_CLAP_MSG];
  App.Packet -> ActiveMessageC;
  App.AMPacket -> ActiveMessageC;
  App.PacketTimeStamp -> ActiveMessageC;

  
  
  App.GlobalTime -> TimeSyncC;
  App.TimeSyncInfo -> TimeSyncC;


}
