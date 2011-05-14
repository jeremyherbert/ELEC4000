#include "ECGHeader.h"
#include <Timer.h>

module patientC {
  uses interface Boot;
  uses interface Leds;
  uses interface Timer<TMilli> as ReadTimer;
  uses interface Timer<TMilli> as HeartBeatTimer;
  uses interface Packet;
  uses interface AMPacket;
  uses interface AMSend as ECGMsgSend;
  uses interface SplitControl as RadioControl;
  uses interface GlobalTime<TMilli>;
} implementation {
  message_t msg;
  
  uint16_t ID;
  uint16_t BEAT_INTERVAL_MS = 15000;
  uint16_t READ_INTERVAL_MS = 15000;



  event void Boot.booted(){
    call RadioControl.start();
    ID = TOS_NODE_ID;

  }

  event void RadioControl.startDone(error_t result){
    call ReadTimer.startPeriodic(READ_INTERVAL_MS);
    call HeartBeatTimer.startPeriodic(BEAT_INTERVAL_MS);
 }

  event void RadioControl.stopDone(error_t result){}

  event void HeartBeatTimer.fired(){
    BEAT_MSG * beat_msg = (BEAT_MSG*)(call Packet.getPayload(&msg, sizeof (BEAT_MSG)));
  	
    beat_msg->NODE_ID = ID;

    call ECGMsgSend.send(AM_BROADCAST_ADDR, &msg, sizeof(BEAT_MSG));

  }


  event void ReadTimer.fired(){
	/*RssiMsg * rssi_msg = (RssiMsg*)(call Packet.getPayload(&msg, sizeof (RssiMsg)));

	Sequence_Number = ++Sequence_Number % 1000;

	
	
	rssi_msg->NODE_ID = ID;
	rssi_msg->seq_num = Sequence_Number;

	if(rssi_msg->NODE_ID == 33){
		call Leds.led0Toggle();
	} else {
		call Leds.led1Toggle();
	}
	


	call RssiMsgSend.send(AM_BROADCAST_ADDR, &msg, sizeof(RssiMsg));  */  
  }

  event void ECGMsgSend.sendDone(message_t *m, error_t error){}
}
