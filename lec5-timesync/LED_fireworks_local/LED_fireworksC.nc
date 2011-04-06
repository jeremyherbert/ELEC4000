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

  event void Boot.booted() {
    call Leds.set(0);
    call Timer.startOneShot(TOS_NODE_ID*period/numNodes);
  }

  enum {STATE_ON, STATE_OFF};
  uint8_t state=STATE_OFF;;

  event void Timer.fired()
  {
    if (state == STATE_OFF){
      state = STATE_ON;
      call Leds.led2On();
      call Timer.startOneShot(period/numNodes);
    }
    else{
      state = STATE_OFF;
      call Leds.led2Off();
      call Timer.startOneShot(period-period/numNodes);
    }
  }
}
