#include "printf.h"
#include <UserButton.h>
#include "NurseHeader.h"

module NurseC {
  uses	{ 

	interface Boot;
	interface Leds;
	
	interface Timer<TMilli> as BuzzTimer;
	interface Notify<button_state_t>;

	interface HplMsp430GeneralIO as io;

	interface Packet;
	interface AMPacket;
        interface Receive;
	interface AMSend as ECGMsgSend;
	interface SplitControl as RadioControl;

  }


} implementation {

 event void Boot.booted(){

		call io.selectIOFunc();
		call io.makeOutput();
		call io.clr();

    call Notify.enable();

    printf("started\n");
    printfflush();

}

 event void Notify.notify (button_state_t state) {
    if ( state == BUTTON_PRESSED) {
	/*if(call io.get()){
		call io.clr();
	} else {
		call io.set();
	} */     
	//call io.toggle();
	 call BuzzTimer.startOneShot(ALARM_INTERVAL);
    }

  }

    event message_t* Receive.receive(message_t* msgPtr, void* payload, uint8_t len)
    {
	ALARM_DATA * packet;

	if(len == sizeof(ALARM_DATA)) {
	packet = (ALARM_DATA*)call Packet.getPayload(msgPtr, 
					sizeof(ALARM_DATA));

		switch(packet->Type){
		case 1: call BuzzTimer.startOneShot(ALARM_INTERVAL);
			break;
		case 2: call io.set();
			break;
		default: call io.clr();
		} 

	}


        return msgPtr;
    }



  event void BuzzTimer.fired(){	
		

	if(call io.get()){
		call io.clr();
	} else {
		call io.set();
	} 
	call BuzzTimer.startOneShot(ALARM_INTERVAL);
}


  event void ECGMsgSend.sendDone(message_t *m, error_t error){}



  event void RadioControl.startDone(error_t result){

 }

  event void RadioControl.stopDone(error_t result){}



}
