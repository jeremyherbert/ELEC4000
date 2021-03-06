#include "ApplicationDefinitions.h"
#include "RssiMessage.h"
#include <Timer.h>

module SendingMoteC {
  uses interface Boot;
  uses interface Leds;
  uses interface Timer<TMilli> as SendTimer;
  uses interface Packet;
  uses interface AMPacket;
  uses interface AMSend as RssiMsgSend;
  uses interface SplitControl as RadioControl;
} implementation {
  message_t msg;
  
  uint16_t ID;
  uint16_t Sequence_Number;

  event void Boot.booted(){
    call RadioControl.start();
    ID = TOS_NODE_ID;
    Sequence_Number = 0;
  }

  event void RadioControl.startDone(error_t result){
    call SendTimer.startPeriodic(SEND_INTERVAL_MS);
  }

  event void RadioControl.stopDone(error_t result){}


  event void SendTimer.fired(){
	RssiMsg * rssi_msg = (RssiMsg*)(call Packet.getPayload(&msg, sizeof (RssiMsg)));

	Sequence_Number = ++Sequence_Number % 1000;

	
	
	rssi_msg->NODE_ID = ID;
	rssi_msg->seq_num = Sequence_Number;

	if(rssi_msg->NODE_ID == 33){
		call Leds.led0Toggle();
	} else {
		call Leds.led1Toggle();
	}
	


	call RssiMsgSend.send(AM_BROADCAST_ADDR, &msg, sizeof(RssiMsg));    
  }

  event void RssiMsgSend.sendDone(message_t *m, error_t error){}
}
