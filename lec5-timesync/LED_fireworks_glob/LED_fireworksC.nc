module LED_fireworksC
{
    uses
    {
        interface GlobalTime<TMilli>;
        interface Timer<TMilli>;
        interface Leds;
        interface Boot;
    }
}
implementation
{
  uint16_t numNodes=4;
  uint16_t period=1024;//ms
  uint16_t ledOnTime;

  event void Boot.booted() {
    ledOnTime = TOS_NODE_ID*period/numNodes;
    call Leds.set(0);
    call Timer.startOneShot(1024);
  }

  enum {STATE_ON, STATE_OFF};
  uint8_t state=STATE_OFF;;

  uint16_t nextLedOnTime(uint32_t currTime){
    uint16_t currentModuloTime = currTime%period;
    if (ledOnTime<currentModuloTime)
      currentModuloTime -= period;
    return ledOnTime-currentModuloTime;
  }

 event void Timer.fired()
  {
    if (state == STATE_OFF){
      state = STATE_ON;
      call Leds.led2On();
      call Timer.startOneShot(period/numNodes);
    }
    else {
      uint32_t currGlobTime = call GlobalTime.getLocalTime();
      call GlobalTime.getGlobalTime(&currGlobTime);
      state = STATE_OFF;
      call Leds.led2Off();
      call Timer.startOneShot(nextLedOnTime(currGlobTime));
    }
  }
}
