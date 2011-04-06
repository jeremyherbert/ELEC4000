configuration LED_fireworksAppC {
}

implementation {
  components MainC, TimeSyncC;
  TimeSyncC.Boot -> MainC;
  MainC.SoftwareInit -> TimeSyncC;

  components LED_fireworksC as App;
  App.GlobalTime -> TimeSyncC;
  App.Boot -> MainC;

  components new TimerMilliC() as TimerC;
  components LedsC;
  App.Timer -> TimerC;
  App.Leds -> LedsC;

}
