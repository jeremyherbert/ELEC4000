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

bool buzz = FALSE;


 event void Boot.booted(){
    	call RadioControl.start();
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
	
		printf("Received Message\n");
		printfflush();
	
		if(len == sizeof(ALARM_DATA)) {
		packet = (ALARM_DATA*)payload;

			switch(packet->Type){
			case 0: 
				buzz = FALSE;
				call io.clr();
				break;
			case 1: buzz = TRUE; 
				call BuzzTimer.startOneShot(ALARM_INTERVAL);
				break;
			default: 
				buzz = FALSE;
				call io.set();
				break;
			
			} 

		}


        return msgPtr;
    }



  event void BuzzTimer.fired(){	
		
	if(buzz == TRUE){
		if(call io.get()){
			call io.clr();
		} else {
			call io.set();
		} 
		call BuzzTimer.startOneShot(ALARM_INTERVAL);
	}
}


  event void ECGMsgSend.sendDone(message_t *m, error_t error){}



  event void RadioControl.startDone(error_t result){

 }

  event void RadioControl.stopDone(error_t result){}



}
